void test_ufs(void);
int ufs_write_raw(uint32_t lba, const void *buf, uint32_t bytes);
int ufs_flush(void);
int ufs_probe_lun(void);
