#!/usr/bin/env python3
"""
make_keepout_mask.py — Nav2 KeepoutFilter 마스크 생성기.

라이다가 물리적으로 못 보는 장애물(책상 상판, 의자 좌판 등)을 '월드 좌표'로
정의하면, 베이스 맵과 동일한 해상도/원점의 keepout 마스크(.pgm + .yaml)를 만든다.
GIMP 픽셀 칠하기 없이 config/keepout_zones.yaml 좌표만 수정→재실행하면 된다.

마스크 픽셀 규약 (trinary):
  흰색(255) = free  → keepout 아님(영향 없음)
  검은색(0) = occupied → KeepoutFilter가 LETHAL로 막음

사용:
  python3 make_keepout_mask.py \
      --base ~/maps/office_clean.yaml \
      --zones <pkg>/config/keepout_zones.yaml \
      --out  <pkg>/map/keepout_mask

베이스 맵을 바꾸면(--base) 마스크 기하도 그 맵에 맞춰 다시 만든다.
"""
import argparse
import os
import sys

import numpy as np
import yaml


def load_map_yaml(path):
    path = os.path.expanduser(path)
    with open(path) as f:
        m = yaml.safe_load(f)
    img = m['image']
    if not os.path.isabs(img):
        img = os.path.join(os.path.dirname(path), img)
    return m, img


def read_pgm_size(pgm_path):
    """P5 PGM에서 (width, height)만 읽는다."""
    with open(pgm_path, 'rb') as f:
        assert f.readline().strip().startswith(b'P5'), 'P5(이진 PGM)만 지원'
        line = f.readline()
        while line.startswith(b'#'):
            line = f.readline()
        w, h = map(int, line.split())
    return w, h


def world_to_px(x, y, origin, res, height):
    """월드(m) → 픽셀(col,row). 이미지는 위→아래, 맵 원점은 좌하단."""
    col = (x - origin[0]) / res
    row = height - (y - origin[1]) / res  # y축 뒤집기
    return col, row


def fill_rect(mask, p1, p2, origin, res, height):
    (x1, y1), (x2, y2) = p1, p2
    c1, r1 = world_to_px(min(x1, x2), max(y1, y2), origin, res, height)  # 좌상
    c2, r2 = world_to_px(max(x1, x2), min(y1, y2), origin, res, height)  # 우하
    c1, c2 = sorted((int(np.floor(c1)), int(np.ceil(c2))))
    r1, r2 = sorted((int(np.floor(r1)), int(np.ceil(r2))))
    c1, r1 = max(c1, 0), max(r1, 0)
    c2, r2 = min(c2, mask.shape[1]), min(r2, mask.shape[0])
    mask[r1:r2, c1:c2] = 0


def fill_circle(mask, center, radius, origin, res, height):
    cx, cy = center
    col0, row0 = world_to_px(cx, cy, origin, res, height)
    rad_px = radius / res
    rr, cc = np.ogrid[:mask.shape[0], :mask.shape[1]]
    inside = (cc - col0) ** 2 + (rr - row0) ** 2 <= rad_px ** 2
    mask[inside] = 0


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--base', required=True, help='베이스 맵 yaml (해상도/원점/크기 상속)')
    ap.add_argument('--zones', required=True, help='keepout_zones.yaml')
    ap.add_argument('--out', required=True, help='출력 경로(확장자 없이). .pgm/.yaml 생성')
    args = ap.parse_args()

    base, base_pgm = load_map_yaml(args.base)
    res = float(base['resolution'])
    origin = [float(v) for v in base['origin'][:2]]
    w, h = read_pgm_size(base_pgm)

    with open(os.path.expanduser(args.zones)) as f:
        zdoc = yaml.safe_load(f) or {}
    zones = zdoc.get('zones', []) or []

    mask = np.full((h, w), 255, dtype=np.uint8)  # 전부 free
    n_applied = 0
    for z in zones:
        kind = z.get('type', 'rect')
        if kind == 'rect':
            fill_rect(mask, z['min'], z['max'], origin, res, h)
        elif kind == 'circle':
            fill_circle(mask, z['center'], float(z['radius']), origin, res, h)
        else:
            print(f"  ! 알 수 없는 zone type '{kind}' 건너뜀: {z}", file=sys.stderr)
            continue
        n_applied += 1
        print(f"  + {z.get('name','(unnamed)')}: {kind} 적용")

    out = os.path.expanduser(args.out)
    out_pgm, out_yaml = out + '.pgm', out + '.yaml'
    with open(out_pgm, 'wb') as f:
        f.write(b'P5\n# keepout mask (make_keepout_mask.py)\n')
        f.write(f'{w} {h}\n255\n'.encode())
        f.write(mask.tobytes())

    mask_yaml = {
        'image': os.path.basename(out_pgm),
        'mode': 'trinary',
        'resolution': res,
        'origin': base['origin'],
        'negate': 0,
        'occupied_thresh': 0.65,
        'free_thresh': 0.25,
    }
    with open(out_yaml, 'w') as f:
        yaml.safe_dump(mask_yaml, f, sort_keys=False)

    occ = int((mask == 0).sum())
    print(f"\n생성: {out_pgm} ({w}x{h}px), {out_yaml}")
    print(f"zone {n_applied}개 적용, keepout 픽셀 {occ}개 ({100*occ/(w*h):.2f}%)")
    if n_applied == 0:
        print("⚠️  zone이 0개다 — keepout_zones.yaml에 실제 좌표를 채워라(현재 마스크는 전부 통행가능).")


if __name__ == '__main__':
    main()
