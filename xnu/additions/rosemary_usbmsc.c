/* MT6785 USB BOT root, polled and read-only. No USB hubs/UAS/hotplug.
 * Hardware sequence derived from shim/usb.c; native XNU port needs HW testing.
 * DMA addresses are physical 64-bit addresses, never kernel virtual pointers.
 */
#include <sys/param.h>
#include <sys/systm.h>
#include <sys/buf.h>
#include <sys/conf.h>
#include <sys/disk.h>
#include <sys/errno.h>
#include <kern/clock.h>
#include <kern/locks.h>
#include <machine/machine_routines.h>
#include <arm/caches_internal.h>
#include <vm/pmap.h>
#include <vm/vm_kern.h>
#include <pexpert/pexpert.h>
#include <libkern/libkern.h>

#if defined(ARM64_BOARD_CONFIG_ROSEMARY)
/* Private ARM APIs are not declared by the BSD exported headers.
 * Same declarations as the existing rosemary_ufs driver. */
extern ppnum_t pmap_find_phys(pmap_t pmap, addr64_t va);
extern vm_offset_t ml_io_map(vm_offset_t phys_addr, vm_size_t size);
#define NT 32u
#define POOL_SIZE 0x100000u
#define CMD 0x0000u
#define EVT 0x1000u
#define ERST 0x2000u
#define DCBAA 0x3000u
#define INPUT 0x4000u
#define OUTPUT 0x5000u
#define EP0 0x6000u
#define BIN 0x7000u
#define BOUT 0x8000u
#define CBW 0x9000u
#define CSW 0xa000u
#define DATA 0x10000u
#define DATA_SIZE 0x10000u
#define SCRATCH_TABLE 0x20000u
#define SCRATCH 0x21000u
#define MAX_SCRATCH 32u

struct usb_ring { uint32_t off, index, cycle; };
static struct {
 vm_offset_t usb, op, rt, db, phy, ippc, infra, gpio, pool;
 uint64_t pa, sectors;
 uint32_t slot, port, speed, in, out, mpsi, mpso, iface, tag, blocksize;
 uint32_t ei, ec, maxslots;
 struct usb_ring cmd, ep0, bin, bout;
 dev_t dev;
 bool ready, failed;
} sc;
static LCK_GRP_DECLARE(usbmsc_group, "rosemary_usbmsc");
static LCK_MTX_DECLARE(usbmsc_lock, &usbmsc_group);
int rosemary_usbmsc_rootdev(dev_t *dev);

static uint32_t rd(vm_offset_t b, uint32_t o) { return *(volatile uint32_t *)(b+o); }
static void wr(vm_offset_t b, uint32_t o, uint32_t v) {
 *(volatile uint32_t *)(b+o)=v; __asm__ volatile("dsb sy" ::: "memory");
}
static void wr64(vm_offset_t b, uint32_t o, uint64_t v) {
 wr(b,o,(uint32_t)v); wr(b,o+4,(uint32_t)(v>>32));
}
static void udelay(uint32_t us) { delay((int)us); }
static uint8_t *mem(uint32_t off) { return (uint8_t *)(sc.pool+off); }
static uint64_t phys(uint32_t off) { return sc.pa+off; }
static void clean(uint32_t off, uint32_t len) {
 clean_dcache64((addr64_t)phys(off),len,TRUE);
 __asm__ volatile("dsb sy" ::: "memory");
}
/* flush before DMA and after device writes, on dedicated cache-line regions.
 * CPU never modifies event entries after initialization. */
static void sync_dma(uint32_t off, uint32_t len) {
 flush_dcache64((addr64_t)phys(off),len,TRUE);
 __asm__ volatile("dsb sy" ::: "memory");
}
static uint32_t le32(const uint8_t *p) {
 return p[0]|((uint32_t)p[1]<<8)|((uint32_t)p[2]<<16)|((uint32_t)p[3]<<24);
}
static uint32_t be32(const uint8_t *p) {
 return ((uint32_t)p[0]<<24)|((uint32_t)p[1]<<16)|((uint32_t)p[2]<<8)|p[3];
}
static void put32(uint8_t *p,uint32_t v) {
 p[0]=(uint8_t)v;p[1]=(uint8_t)(v>>8);p[2]=(uint8_t)(v>>16);p[3]=(uint8_t)(v>>24);
}
static int bits(vm_offset_t b,uint32_t o,uint32_t mask,uint32_t want,uint32_t us) {
 for(uint32_t n=0;n<us;n+=10) { if((rd(b,o)&mask)==want)return 0;udelay(10); }
 return ETIMEDOUT;
}

/* GPIO bit-bang helpers from the proven shim are inserted below. */
static uint32_t gpio_rd(uintptr_t p) { return rd((vm_offset_t)p,0); }
static void gpio_wr(uintptr_t p,uint32_t v) { wr((vm_offset_t)p,0,v); }
#define BB_SCL	28u
#define BB_SDA	29u

static inline uintptr_t g_dir(unsigned p)  { return sc.gpio + 0x0000 + (p >> 5) * 0x10; }
static inline uintptr_t g_dout(unsigned p) { return sc.gpio + 0x0100 + (p >> 5) * 0x10; }
static inline uintptr_t g_din(unsigned p)  { return sc.gpio + 0x0200 + (p >> 5) * 0x10; }
static inline uintptr_t g_mode(unsigned p) { return sc.gpio + 0x0300 + (p >> 3) * 0x10; }

static void g_out(unsigned p, int v)
{
	uintptr_t r = g_dout(p);
	uint32_t b = 1u << (p & 31), d = gpio_rd(r);
	gpio_wr(r, v ? (d | b) : (d & ~b));
}

static int g_in(unsigned p) { return (int)((gpio_rd(g_din(p)) >> (p & 31)) & 1u); }

static void g_dirset(unsigned p, int out)
{
	uintptr_t r = g_dir(p);
	uint32_t b = 1u << (p & 31), d = gpio_rd(r);
	gpio_wr(r, out ? (d | b) : (d & ~b));
}

static void g_mode_gpio(unsigned p)
{
	uintptr_t r = g_mode(p);
	uint32_t sh = (p & 7) * 4, d = gpio_rd(r);
	gpio_wr(r, d & ~(0xfu << sh));
}

static void bb_d(void) { udelay(4); }
static void bb_scl(int v) { g_out(BB_SCL, v); }
static void bb_sda(int v) { g_out(BB_SDA, v); }

static void bb_start(void)
{
	g_dirset(BB_SDA, 1);
	bb_sda(1); bb_scl(1); bb_d();
	bb_sda(0); bb_d();
	bb_scl(0); bb_d();
}

static void bb_stop(void)
{
	g_dirset(BB_SDA, 1);
	bb_sda(0); bb_d();
	bb_scl(1); bb_d();
	bb_sda(1); bb_d();
}

static int bb_write(uint8_t b)
{
	int i, ack;

	for (i = 7; i >= 0; i--) {
		bb_sda((b >> i) & 1);
		bb_d();
		bb_scl(1); bb_d();
		bb_scl(0); bb_d();
	}
	g_dirset(BB_SDA, 0);		/* release for ACK */
	bb_scl(1); bb_d();
	ack = !g_in(BB_SDA);
	bb_scl(0); bb_d();
	g_dirset(BB_SDA, 1);
	return ack;
}

static uint8_t bb_read(int nack)
{
	uint8_t v = 0;
	int i;

	g_dirset(BB_SDA, 0);
	for (i = 7; i >= 0; i--) {
		bb_d();
		bb_scl(1); bb_d();
		v = (uint8_t)((v << 1) | (g_in(BB_SDA) & 1));
		bb_scl(0);
	}
	g_dirset(BB_SDA, 1);
	bb_sda(nack ? 1 : 0);
	bb_d();
	bb_scl(1); bb_d();
	bb_scl(0); bb_d();
	bb_sda(1);
	return v;
}

static int bb_rd8(uint8_t slave, uint8_t reg, uint8_t *out)
{
	int ack;

	bb_start();
	ack = bb_write((uint8_t)(slave << 1));
	if (!ack) { bb_stop(); return -1; }
	ack = bb_write(reg);
	if (!ack) { bb_stop(); return -2; }
	udelay(150);
	bb_start();
	ack = bb_write((uint8_t)((slave << 1) | 1));
	if (!ack) { bb_stop(); return -3; }
	*out = bb_read(1);
	bb_stop();
	return 0;
}

static void bb_pins_init(void)
{
	g_mode_gpio(BB_SCL);
	g_mode_gpio(BB_SDA);
	g_dirset(BB_SCL, 1);
	g_dirset(BB_SDA, 1);
	bb_scl(1); bb_sda(1);
	udelay(200);
}

static int bb_wr8(uint8_t slave,uint8_t reg,uint8_t value) {
 bb_start();
 if(!bb_write((uint8_t)(slave<<1))||!bb_write(reg)||!bb_write(value)) {bb_stop();return EIO;}
 bb_stop();return 0;
}


static int power_up(void) {
 printf("usbmsc: vbus start\n");
 uint8_t c11=0,c1d=0,check=0;
 bb_pins_init();
 if(bb_rd8(0x34,0x11,&c11)||bb_rd8(0x34,0x1d,&c1d))return EIO;
 if(bb_wr8(0x34,0x1d,c1d|0x80)||bb_wr8(0x34,0x15,0x5c)||
    bb_wr8(0x34,0x11,c11&~1u))return EIO;
 udelay(150000);
 if(bb_wr8(0x34,0x11,c11|1u))return EIO;
 udelay(250000);
 if(bb_rd8(0x34,0x11,&check)||!(check&1))return EIO;
 if(bb_wr8(0x4e,0x1a,5)||bb_wr8(0x4e,4,0x10))return EIO;
 wr(sc.infra,0xa8,(1u<<1)|(1u<<31));wr(sc.infra,0x84,1u<<8);
 wr(sc.ippc,0,rd(sc.ippc,0)&~1u);wr(sc.ippc,4,rd(sc.ippc,4)&~1u);
 wr(sc.phy,0xb04,rd(sc.phy,0xb04)|(1u<<29));udelay(250);
 wr(sc.phy,0x368,rd(sc.phy,0x368)&~((3u<<30)|(1u<<26)|(1u<<18)));
 wr(sc.phy,0x36c,rd(sc.phy,0x36c)&~(1u<<16));
 wr(sc.phy,0x320,(rd(sc.phy,0x320)&~((1u<<9)|(1u<<8)|(1u<<16)|(1u<<17)))|(1u<<18));
 wr(sc.phy,0x318,rd(sc.phy,0x318)&~((1u<<23)|(1u<<20)));udelay(800);
 wr(sc.phy,0x36c,0x2614);wr(sc.ippc,0x50,4);udelay(20000);
 printf("usbmsc: vbus done\n");
 return 0;
}
static void init_ring(struct usb_ring *r,uint32_t off) {
 r->off=off;r->index=0;r->cycle=1;bzero(mem(off),NT*16);
 /* All entries initially unowned (cycle=0). Link published at wrap. */
 clean(off,NT*16);
}
static uint64_t push(struct usb_ring *r,uint64_t param,uint32_t status,uint32_t control) {
 uint32_t off=r->off+r->index*16;
 uint32_t *t=(uint32_t *)mem(off);
 if(r->index==NT-2) {
  uint32_t *link=(uint32_t *)mem(r->off+(NT-1)*16);
  link[0]=(uint32_t)phys(r->off);link[1]=(uint32_t)(phys(r->off)>>32);
  link[2]=0;link[3]=(6u<<10)|2u|r->cycle;
  clean(r->off+(NT-1)*16,16);
 }
 t[0]=(uint32_t)param;t[1]=(uint32_t)(param>>32);t[2]=status;
 t[3]=(control&~1u)|r->cycle;clean(off,16);
 if(++r->index==NT-1){r->index=0;r->cycle^=1;}
 return phys(off);
}
/* Wait for exact command/transfer pointer. Ignore port-status notifications.
 * Event entries are not cleared; consumer cycle toggles at end of segment. */
static int event(uint32_t type,uint64_t ptr,uint32_t dci,uint32_t result[4]) {
 for(uint32_t elapsed=0;elapsed<4000000;elapsed+=10) {
  uint32_t off=EVT+sc.ei*16;
  sync_dma(off,16);
  volatile uint32_t *e=(volatile uint32_t *)mem(off);
  if((e[3]&1)!=sc.ec){udelay(10);continue;}
  uint32_t v[4]={e[0],e[1],e[2],e[3]};
  if(++sc.ei==NT){sc.ei=0;sc.ec^=1;}
  wr64(sc.rt,0x18,phys(EVT+sc.ei*16)|8u);
  uint32_t et=(v[3]>>10)&63;
  if(et==37){printf("usbmsc: host event cc=%u\n",v[2]>>24);return EIO;}
  if(et!=type)continue;
  uint64_t ep=((uint64_t)v[1]<<32)|v[0];
  if(type==32 && (((v[3]>>24)&255)!=sc.slot || ((v[3]>>16)&31)!=dci))continue;
  if((ep&~15ull)!=(ptr&~15ull)) {
   if((v[2]>>24)!=1){printf("usbmsc: intermediate cc=%u dci=%u\n",v[2]>>24,dci);return EIO;}
   continue;
  }
  for(unsigned i=0;i<4;i++)result[i]=v[i];
  if((v[2]>>24)!=1){printf("usbmsc: completion type=%u dci=%u cc=%u\n",type,dci,v[2]>>24);return EIO;}
  return 0;
 }
 printf("usbmsc: timeout type=%u dci=%u trb=%llx\n",type,dci,(unsigned long long)ptr);
 return ETIMEDOUT;
}
static int command(uint32_t type,uint64_t param,uint32_t extra,uint32_t *slot) {
 uint32_t e[4];uint64_t p=push(&sc.cmd,param,0,(type<<10)|extra);
 wr(sc.db,0,0);int err=event(33,p,0,e);
 if(!err && slot)*slot=e[3]>>24;
 return err;
}
static int start_controller(void) {
 printf("usbmsc: power start\n");
 int err=power_up();if(err)return err;
 printf("usbmsc: power done\n");
 uint32_t cap=rd(sc.usb,0),hcs=rd(sc.usb,4),hcc=rd(sc.usb,0x10);
 uint32_t caplen=cap&255,ports=hcs>>24;
 if(caplen!=0x20 || !ports || (hcc&4))return ENOTSUP;
 sc.op=sc.usb+caplen;
 uint32_t dbo=rd(sc.usb,0x14)&~3u,rto=rd(sc.usb,0x18)&~31u;
 if(dbo<0x20||dbo>0x3000||rto<0x20||rto>0x3000)return ENOTSUP;
 sc.db=sc.usb+dbo;sc.rt=sc.usb+rto+0x20;
 if(!(hcc&1) && phys(POOL_SIZE-1)>0xffffffffull)return ENOTSUP;
 if(bits(sc.op,4,1u<<11,0,200000))return ETIMEDOUT;
 wr(sc.op,0,0);if(bits(sc.op,4,1,1,200000))return ETIMEDOUT;
 wr(sc.op,0,2);if(bits(sc.op,0,2,0,200000)||bits(sc.op,4,1u<<11,0,200000))return ETIMEDOUT;
 if(!(rd(sc.op,8)&1))return ENOTSUP;
 bzero((void *)sc.pool,POOL_SIZE);
 uint32_t hcs2=rd(sc.usb,8),scratch=((hcs2>>27)&31)|((hcs2>>16)&0x3e0);
 if(scratch>MAX_SCRATCH)return ENOTSUP;
 uint64_t *dc=(uint64_t *)mem(DCBAA);
 if(scratch) {
  dc[0]=phys(SCRATCH_TABLE);
  uint64_t *st=(uint64_t *)mem(SCRATCH_TABLE);
  for(unsigned i=0;i<scratch;i++)st[i]=phys(SCRATCH+i*4096);
 }
 sc.maxslots=hcs&255;if(!sc.maxslots)return ENODEV;
 init_ring(&sc.cmd,CMD);init_ring(&sc.ep0,EP0);
 init_ring(&sc.bin,BIN);init_ring(&sc.bout,BOUT);
 sc.ei=0;sc.ec=1;
 uint32_t *erst=(uint32_t *)mem(ERST);
 erst[0]=(uint32_t)phys(EVT);erst[1]=(uint32_t)(phys(EVT)>>32);erst[2]=NT;
 sync_dma(0,POOL_SIZE);
 wr64(sc.op,0x18,phys(CMD)|1);wr64(sc.op,0x30,phys(DCBAA));wr(sc.op,0x38,sc.maxslots);
 wr(sc.rt,0,2);wr(sc.rt,4,0);wr(sc.rt,8,1);
 wr64(sc.rt,0x10,phys(ERST));wr64(sc.rt,0x18,phys(EVT));wr(sc.op,0,1);
 if(bits(sc.op,4,1,0,100000))return ETIMEDOUT;
 for(uint32_t i=0;i<ports;i++) {
  uint32_t o=0x400+i*16,ps=rd(sc.op,o);
  wr(sc.op,o,(ps&((15u<<5)|(1u<<9)))|(1u<<9));udelay(20000);
  ps=rd(sc.op,o);if(!(ps&1))continue;
  wr(sc.op,o,(ps&((15u<<5)|(1u<<9)))|16u);
  if(bits(sc.op,o,16,0,500000))continue;
  udelay(20000);ps=rd(sc.op,o);
  if(!(ps&2))continue;
  sc.port=i+1;sc.speed=(ps>>10)&15;
  /* First implementation is USB2 high speed; no hub/TT or SuperSpeed. */
  if(sc.speed!=3)return ENOTSUP;
  printf("usbmsc: port=%u speed=%u dma=%llx\n",sc.port,sc.speed,(unsigned long long)sc.pa);
  return 0;
 }
 return ENODEV;
}
static void ep_context(uint32_t dci,uint32_t type,uint32_t mps,uint32_t ringoff) {
 uint32_t *e=(uint32_t *)mem(INPUT+(dci+1)*32);
 e[1]=(mps<<16)|(type<<3)|(3u<<1);
 e[2]=(uint32_t)phys(ringoff)|1;e[3]=(uint32_t)(phys(ringoff)>>32);
 e[4]=type==4?8:512;
}
static int control(uint8_t reqtype,uint8_t req,uint16_t value,uint16_t index,uint16_t length) {
 uint32_t e[4];
 uint64_t setup=(uint64_t)reqtype|((uint64_t)req<<8)|((uint64_t)value<<16)|
                ((uint64_t)index<<32)|((uint64_t)length<<48);
 if(length){bzero(mem(DATA),length);sync_dma(DATA,length);}
 push(&sc.ep0,setup,8,(2u<<10)|(1u<<6)|(length?(3u<<16):0));
 if(length)push(&sc.ep0,phys(DATA),length,(3u<<10)|(1u<<16)|(1u<<2));
 uint64_t p=push(&sc.ep0,0,0,(4u<<10)|(1u<<5)|(length?0:(1u<<16)));
 wr(sc.db,sc.slot*4,1);int err=event(32,p,1,e);
 if(length)sync_dma(DATA,length);
 return err;
}
static int enumerate(void) {
 printf("usbmsc: enumerate start\n");
 int err=start_controller();if(err)return err;
 printf("usbmsc: controller ready\n");
 if((err=command(9,0,0,&sc.slot)))return err;
 if(!sc.slot||sc.slot>sc.maxslots)return EIO;
 ((uint64_t *)mem(DCBAA))[sc.slot]=phys(OUTPUT);clean(DCBAA,4096);
 bzero(mem(INPUT),4096);
 uint32_t *ctx=(uint32_t *)mem(INPUT);
 ctx[1]=3;ctx[8]=(sc.speed<<20)|(1u<<27);ctx[9]=sc.port<<16;
 ep_context(1,4,64,EP0);clean(INPUT,4096);
 if((err=command(11,phys(INPUT),sc.slot<<24,NULL)))return err;
 if((err=control(0x80,6,0x100,0,18)))return err;
 uint8_t *d=mem(DATA);
 if(d[0]!=18||d[1]!=1||d[7]!=64)return ENOTSUP;
 printf("usbmsc: vid=%04x pid=%04x slot=%u\n",d[8]|d[9]<<8,d[10]|d[11]<<8,sc.slot);
 if((err=control(0x80,6,0x200,0,9)))return err;
 uint32_t total=d[2]|d[3]<<8,config=d[5];
 if(d[1]!=2||total<9||total>4096||!config)return ENOTSUP;
 if((err=control(0x80,6,0x200,0,(uint16_t)total)))return err;
 bool selected=false,active=false;
 for(uint32_t i=0;i+2<=total;) {
  uint32_t n=d[i];if(n<2||n>total-i)return EIO;
  if(d[i+1]==4&&n>=9) {
   active=false;
   if(!selected&&d[i+3]==0&&d[i+5]==8&&d[i+6]==6&&d[i+7]==0x50) {
    selected=true;active=true;sc.iface=d[i+2];
   }
  } else if(active&&d[i+1]==5&&n>=7&&(d[i+3]&3)==2) {
   uint32_t ep=d[i+2]&15,mps=(d[i+4]|d[i+5]<<8)&0x7ff;
   if(!ep||mps!=512)return ENOTSUP;
   if(d[i+2]&128){if(sc.in)return ENOTSUP;sc.in=ep*2+1;sc.mpsi=mps;}
   else {if(sc.out)return ENOTSUP;sc.out=ep*2;sc.mpso=mps;}
  }
  i+=n;
 }
 if(!sc.in||!sc.out)return ENOTSUP;
 if((err=control(0,9,(uint16_t)config,0,0)))return err;
 sync_dma(OUTPUT,1024);bzero(mem(INPUT),4096);
 ctx[1]=1u|(1u<<sc.in)|(1u<<sc.out);
 bcopy(mem(OUTPUT),mem(INPUT+32),32);
 uint32_t last=sc.in>sc.out?sc.in:sc.out;
 ctx[8]=(ctx[8]&~(31u<<27))|(last<<27);
 ep_context(sc.in,6,sc.mpsi,BIN);ep_context(sc.out,2,sc.mpso,BOUT);
 clean(INPUT,4096);
 if((err=command(12,phys(INPUT),sc.slot<<24,NULL)))return err;
 printf("usbmsc: BOT endpoints in=%u out=%u\n",sc.in,sc.out);
 return 0;
}
static int bulk(bool in,uint32_t off,uint32_t size,uint32_t *actual) {
 if(!size||size>DATA_SIZE||off>POOL_SIZE-size)return EINVAL;
 struct usb_ring *r=in?&sc.bin:&sc.bout;
 uint32_t dci=in?sc.in:sc.out,e[4];
 if(in)sync_dma(off,size);else clean(off,size);
 uint64_t p=push(r,phys(off),size,(1u<<10)|(1u<<5));
 wr(sc.db,sc.slot*4,dci);int err=event(32,p,dci,e);
 if(in)sync_dma(off,size);
 if(err)return err;
 uint32_t remaining=e[2]&0xffffff;
 if(remaining>size)return EIO;
 *actual=size-remaining;
 return 0;
}
/* Single command at a time. On malformed/failed transport, fail closed:
 * do not reuse an uncertain ring or issue a second CBW. No write commands. */
static int bot(const uint8_t *cdb,uint32_t cdblen,uint32_t length) {
 if(sc.failed||length>DATA_SIZE||!cdblen||cdblen>16)return EIO;
 uint32_t tag=++sc.tag,done=0;uint8_t *cbw=mem(CBW),*csw=mem(CSW);
 bzero(cbw,64);bzero(csw,64);
 put32(cbw,0x43425355);put32(cbw+4,tag);put32(cbw+8,length);
 cbw[12]=0x80;cbw[13]=0;cbw[14]=(uint8_t)cdblen;bcopy(cdb,cbw+15,cdblen);
 int err=bulk(false,CBW,31,&done);
 if(err||done!=31)goto transport_fail;
 if(length){err=bulk(true,DATA,length,&done);if(err||done!=length)goto transport_fail;}
 err=bulk(true,CSW,13,&done);
 if(err||done!=13||le32(csw)!=0x53425355||le32(csw+4)!=tag||
    le32(csw+8)>length||csw[12]>2)goto transport_fail;
 if(csw[12]==2)goto transport_fail;
 if(csw[12]==1)return EIO;
 if(le32(csw+8))goto transport_fail;
 return 0;
transport_fail:
 sc.failed=true;printf("usbmsc: BOT transport stopped tag=%u err=%d bytes=%u\n",tag,err,done);
 return err?err:EIO;
}
static int scsi(const uint8_t *cdb,uint32_t cdblen,uint32_t length) {
 for(unsigned attempt=0;attempt<3;attempt++) {
  int err=bot(cdb,cdblen,length);
  if(!err||sc.failed)return err;
  uint8_t sense[6]={3,0,0,0,18,0};
  err=bot(sense,6,18);if(err)return err;
  uint8_t *s=mem(DATA);uint32_t response=s[0]&127;
  if(response!=0x70&&response!=0x71)return EIO;
  uint32_t key=s[2]&15,asc=s[12],ascq=s[13];
  printf("usbmsc: sense key=%u asc=%02x ascq=%02x\n",key,asc,ascq);
  if(key!=6 && !(key==2&&asc==4&&ascq==1))return EIO;
  udelay(200000);
 }
 return EIO;
}
static int read_blocks(uint64_t lba,uint32_t count) {
 if(!count||count>DATA_SIZE/512||lba>0xffffffffull||lba>=sc.sectors||count>sc.sectors-lba)return EINVAL;
 uint8_t c[10]={0x28,0,(uint8_t)(lba>>24),(uint8_t)(lba>>16),(uint8_t)(lba>>8),(uint8_t)lba,0,(uint8_t)(count>>8),(uint8_t)count,0};
 return scsi(c,10,count*512);
}
static int probe_disk(void) {
 uint8_t inquiry[6]={0x12,0,0,0,36,0},cap[10]={0x25};
 int err=scsi(inquiry,6,36);if(err)return err;
 if((mem(DATA)[0]&31)!=0)return ENOTSUP;
 err=scsi(cap,10,8);if(err)return err;
 uint32_t last=be32(mem(DATA)),size=be32(mem(DATA)+4);
 if(last==0xffffffffu||size!=512)return ENOTSUP;
 sc.sectors=(uint64_t)last+1;sc.blocksize=512;
 printf("usbmsc: capacity=%llu sectors size=%u\n",(unsigned long long)sc.sectors,size);
 if((err=read_blocks(2,1)))return err;
 uint8_t *v=mem(DATA);
 if(v[0]!='H'||(v[1]!='X'&&v[1]!='+'))return ENODEV;
 uint32_t bs=be32(v+40),blocks=be32(v+44);
 if(bs<512||bs>65536||(bs&(bs-1))||!blocks||((uint64_t)bs*blocks)>sc.sectors*512)return EIO;
 printf("usbmsc: HFS%c block=%u blocks=%u (read-only)\n",v[1],bs,blocks);
 return 0;
}
static void usbmsc_strategy(struct buf *bp) {
 uint64_t bytes=buf_count(bp);daddr64_t block=buf_blkno(bp);int err=0;
 buf_setresid(bp,buf_count(bp));
 lck_mtx_lock(&usbmsc_lock);
 if(!sc.ready||sc.failed){err=ENXIO;goto out;}
 if(!(buf_flags(bp)&B_READ)){err=EROFS;goto out;}
 /* buf_blkno follows the logical block size negotiated by HFS. */
 uint64_t factor=sc.blocksize/512;
 if(block<0||(bytes&511)||((uint64_t)block>sc.sectors/factor)){err=EINVAL;goto out;}
 uint64_t lba=(uint64_t)block*factor;
 if(bytes/512>sc.sectors-lba){err=EINVAL;goto out;}
 uint8_t *dest=(uint8_t *)(uintptr_t)buf_dataptr(bp);
 if(!dest&&bytes){err=EINVAL;goto out;}
 while(bytes) {
  uint32_t n=bytes>DATA_SIZE?DATA_SIZE:(uint32_t)bytes;
  err=read_blocks(lba,n/512);if(err)break;
  bcopy(mem(DATA),dest,n);dest+=n;lba+=n/512;bytes-=n;buf_setresid(bp,(int)bytes);
 }
out:
 lck_mtx_unlock(&usbmsc_lock);if(err)buf_seterror(bp,err);buf_biodone(bp);
}
static int usbmsc_open(__unused dev_t dev,__unused int f,__unused int t,__unused struct proc *p) {return sc.ready?0:ENXIO;}
static int usbmsc_close(__unused dev_t dev,__unused int f,__unused int t,__unused struct proc *p) {return 0;}
static int usbmsc_ioctl(__unused dev_t dev,u_long cmd,caddr_t data,__unused int flags,__unused struct proc *p) {
 uint32_t *v=(uint32_t *)data;uint64_t *q=(uint64_t *)data;int err=0;
 lck_mtx_lock(&usbmsc_lock);
 switch(cmd) {
 case DKIOCGETBLOCKSIZE:*v=sc.blocksize;break;
 case DKIOCGETPHYSICALBLOCKSIZE:*v=512;break;
 case DKIOCSETBLOCKSIZE:
  if(*v<512||*v>4096||(*v&(*v-1)))err=EINVAL;else sc.blocksize=*v;break;
 case DKIOCGETBLOCKCOUNT:*q=sc.sectors*512/sc.blocksize;break;
 case DKIOCISWRITABLE:*v=0;break;
 case DKIOCSYNCHRONIZE:case DKIOCSYNCHRONIZECACHE:break;
 case DKIOCGETMAXBLOCKCOUNTREAD:*q=DATA_SIZE/sc.blocksize;break;
 case DKIOCGETMAXBLOCKCOUNTWRITE:*q=0;break;
 case DKIOCGETMAXSEGMENTCOUNTREAD:case DKIOCGETMAXSEGMENTCOUNTWRITE:*q=1;break;
 default:err=ENOTTY;
 }
 lck_mtx_unlock(&usbmsc_lock);return err;
}
static int usbmsc_psize(__unused dev_t dev){return sc.sectors>0x7fffffff?0x7fffffff:(int)sc.sectors;}
static struct bdevsw usbmsc_bdevsw={.d_open=usbmsc_open,.d_close=usbmsc_close,
 .d_strategy=usbmsc_strategy,.d_ioctl=usbmsc_ioctl,.d_psize=usbmsc_psize,.d_type=D_DISK};
int rosemary_usbmsc_rootdev(dev_t *dev) {
 int enabled=1,err;PE_parse_boot_argn("usbmsc",&enabled,sizeof(enabled));
 printf("usbmsc: root entry enabled=%d\n",enabled);
 if(!enabled)return ENXIO;
 lck_mtx_lock(&usbmsc_lock);
 printf("usbmsc: root lock acquired\n");
 if(sc.ready){*dev=sc.dev;lck_mtx_unlock(&usbmsc_lock);return 0;}
 if(sc.pool){err=ENXIO;goto out;}
 sc.usb=ml_io_map(0x11200000,0x4000);sc.ippc=sc.usb?sc.usb+0x3e00:0;
 sc.phy=ml_io_map(0x11e40000,0x1000);sc.infra=ml_io_map(0x10001000,0x1000);
 sc.gpio=ml_io_map(0x10005000,0x1000);
 if(!sc.usb||!sc.ippc||!sc.phy||!sc.infra||!sc.gpio){err=ENOMEM;goto out;}
 printf("usbmsc: mmio mapped\n");
 if(kmem_alloc_contig(kernel_map,&sc.pool,POOL_SIZE,0xffff,0,0,
    KMA_DATA|KMA_ZERO|KMA_PERMANENT,VM_KERN_MEMORY_IOKIT)!=KERN_SUCCESS){err=ENOMEM;goto out;}
 ppnum_t pn=pmap_find_phys(kernel_pmap,(addr64_t)sc.pool);
 sc.pa=ptoa_64(pn);
 printf("usbmsc: dma allocated va=%llx pa=%llx\n",(unsigned long long)sc.pool,(unsigned long long)sc.pa);
 if(!pn||(sc.pa&0xffff)){err=ENOMEM;goto out;}
 for(uint32_t off=0;off<POOL_SIZE;off+=PAGE_SIZE)
  if(ptoa_64(pmap_find_phys(kernel_pmap,(addr64_t)(sc.pool+off)))!=sc.pa+off){err=ENOMEM;goto out;}
 printf("usbmsc: native xHCI/BOT probe\n");
 err=enumerate();if(err)goto out;
 err=probe_disk();if(err)goto out;
 int majorno=bdevsw_add(-1,&usbmsc_bdevsw);
 if(majorno<0){err=ENODEV;goto out;}
 sc.dev=makedev(majorno,0);sc.ready=true;*dev=sc.dev;
 printf("usbmsc: usbroot0 ready, read-only, major=%d\n",majorno);
out:
 if(err) {
  sc.failed=true;printf("usbmsc: attach failed errno=%d\n",err);
  if(sc.op) {wr(sc.op,0,0);(void)bits(sc.op,4,1,1,200000);}
 }
 lck_mtx_unlock(&usbmsc_lock);return err;
}
#endif
