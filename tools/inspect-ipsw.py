#!/usr/bin/env python3
"""Inspect IPSW restore identities without extracting or modifying anything."""
import argparse
import json
import plistlib
import zipfile


def inspect(path):
    with zipfile.ZipFile(path) as archive:
        manifest = plistlib.loads(archive.read('BuildManifest.plist'))
        names = set(archive.namelist())
        identities = []
        for index, identity in enumerate(manifest.get('BuildIdentities', [])):
            images = {}
            for key, component in identity.get('Manifest', {}).items():
                member = component.get('Info', {}).get('Path', '')
                if member.lower().endswith('.dmg'):
                    images[key] = {'path': member, 'present': member in names}
            identities.append({'index': index, 'info': identity.get('Info', {}),
                               'images': images})
        return {'version': manifest.get('ProductVersion'),
                'build': manifest.get('ProductBuildVersion'),
                'products': manifest.get('SupportedProductTypes'),
                'identities': identities}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('ipsw')
    args = parser.parse_args()
    print(json.dumps(inspect(args.ipsw), indent=2, ensure_ascii=False, default=str))


if __name__ == '__main__':
    main()
