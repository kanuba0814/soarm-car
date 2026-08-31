import argparse
import json
from pathlib import Path

from ament_index_python.packages import get_package_share_directory


def matches_category(record, category):
    return not category or category.lower() in record.get('category', '').lower()


def main():
    parser = argparse.ArgumentParser(description='Query an installed HM3DSem apartment sidecar.')
    parser.add_argument('--world', choices=('apartment_a', 'apartment_b'), required=True)
    parser.add_argument('--category', help='Case-insensitive substring matched against room and object categories.')
    parser.add_argument('--instance-id', type=int)
    parser.add_argument('--dynamic-only', action='store_true')
    parser.add_argument('--semantic-file', type=Path, help='Override the installed sidecar path.')
    args = parser.parse_args()

    path = args.semantic_file
    if path is None:
        share = Path(get_package_share_directory('soarm_simulation'))
        path = share / 'semantics' / (args.world + '.json')
    data = json.loads(path.read_text())
    if data.get('source_scene') is None:
        raise RuntimeError('invalid semantic sidecar: ' + str(path))

    rooms = [room for room in data.get('rooms', []) if matches_category(room, args.category)]
    objects = []
    for record in data.get('objects', []):
        if not matches_category(record, args.category):
            continue
        if args.instance_id is not None and record.get('instance_id') != args.instance_id:
            continue
        if args.dynamic_only and not record.get('dynamic_model_required', False):
            continue
        objects.append(record)
    if args.instance_id is not None or args.dynamic_only:
        rooms = []
    print(json.dumps({
        'world': args.world,
        'source_scene': data['source_scene'],
        'rooms': rooms,
        'objects': objects,
    }, indent=2, sort_keys=True))


if __name__ == '__main__':
    main()
