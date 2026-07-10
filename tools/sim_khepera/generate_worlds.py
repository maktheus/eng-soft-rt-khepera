"""Gera mundos JSON para o simulador OpenGL do Khepera.

O arquivo gerado e simples de editar:

{
  "scenarios": [
    {
      "name": "case_0001_single_pair",
      "difficulty": 1,
      "goal": [2200, 0],
      "bounds": [-450, 2700, -900, 900],
      "obstacles": [
        {"name": "box_1", "rect": [950, 0, 220, 320]}
      ]
    }
  ]
}

Todas as unidades estao em milimetros.
"""

import argparse
import json
from pathlib import Path


DEFAULT_COUNT = 1000
OUT = Path(__file__).with_name("worlds_1000.json")


def rect(name, x, y, w, h):
    return {"name": name, "rect": [round(x, 1), round(y, 1), round(w, 1), round(h, 1)]}


def bounds_for(goal_x, goal_y, margin=650):
    ymin = min(-900, goal_y - margin)
    ymax = max(900, goal_y + margin)
    return [-450, round(goal_x + 520, 1), round(ymin, 1), round(ymax, 1)]


def scenario(name, difficulty, goal, bounds, obstacles):
    return {
        "name": f"case_{difficulty:04d}_{name}",
        "difficulty": difficulty,
        "goal": [round(goal[0], 1), round(goal[1], 1)],
        "bounds": bounds,
        "obstacles": obstacles[:64],
    }


def split_counts(total):
    ratios = [0.15, 0.20, 0.20, 0.25]
    counts = [max(1, int(total * r)) for r in ratios]
    used = sum(counts)
    counts.append(max(1, total - used))
    while sum(counts) > total:
        idx = max(range(len(counts)), key=lambda i: counts[i])
        counts[idx] -= 1
    while sum(counts) < total:
        counts[-1] += 1
    return counts


def single_or_pair(i, local, span):
    gx = 1900 + local * 18
    gy = ((local % 7) - 3) * 55
    size_boost = min(90, local * 0.55)
    obs = [
        rect(
            "box_main",
            gx * 0.42,
            gy * 0.35,
            170 + (local % 5) * 18 + size_boost * 0.15,
            230 + (local % 4) * 28 + size_boost * 0.22,
        )
    ]
    if local > span * 0.45:
        obs.append(rect("box_second", gx * 0.68, gy + (300 if local % 2 else -300), 170 + (local % 4) * 20, 220))
    return scenario("single_pair", i, (gx, gy), bounds_for(gx, gy), obs)


def slalom(i, local, _span):
    count = 3 + min(9, local // 25)
    gx = 2500 + local * 18
    gy = 180 + ((local % 5) - 2) * 45
    amp = 270 + min(330, local * 1.3)
    obs = []
    start_x = 520
    step = max(330, (gx - 860) / max(1, count))
    for k in range(count):
        side = -1 if k % 2 == 0 else 1
        x = start_x + k * step
        y = gy * 0.22 + side * (amp + (k % 3) * 34)
        w = 150 + ((i + k) % 4) * 18
        h = 260 + ((i + 2 * k) % 5) * 26
        obs.append(rect(f"slalom_{k+1}", x, y, w, h))
    return scenario(f"slalom_{count}", i, (gx, gy), bounds_for(gx, gy, margin=1050), obs)


def gates(i, local, _span):
    gate_count = 2 + min(7, local // 28)
    gx = 3000 + local * 17
    gy = ((local % 7) - 3) * 50
    gap = max(430, 650 - local * 0.72)
    obs = []
    step = max(560, (gx - 1020) / max(1, gate_count))
    for k in range(gate_count):
        x = 650 + k * step
        height = 470 + (k % 3) * 70
        obs.append(rect(f"gate_{k+1}_top", x, gy + gap / 2 + height / 2, 190, height))
        obs.append(rect(f"gate_{k+1}_bottom", x, gy - gap / 2 - height / 2, 190, height))
        if k % 2 == 1 and local > 40:
            obs.append(rect(f"offset_{k+1}", x + 300, gy + (300 if local % 2 else -300), 160, 230))
    return scenario(f"gates_gap_{int(gap)}", i, (gx, gy), bounds_for(gx, gy, margin=1200), obs)


def mixed_lab(i, local, _span):
    gx = 3400 + local * 16
    gy = 520 + ((local % 8) - 3) * 65
    count = 8 + min(28, local // 10)
    obs = []
    for k in range(count):
        x = 520 + k * ((gx - 1100) / max(1, count - 1))
        base = ((k * 37 + local * 19) % 1120) - 560
        y = base + 0.20 * gy
        line_y = gy * (x / gx)
        if abs(y - line_y) < 680:
            sign = 1 if (k + local) % 2 else -1
            y = line_y + sign * (760 + ((k + local) % 4) * 42)
        w = 130 + ((k + local) % 5) * 18
        h = 165 + ((2 * k + local) % 6) * 23
        obs.append(rect(f"lab_{k+1}", x, y, w, h))
    return scenario(f"mixed_lab_{count}", i, (gx, gy), bounds_for(gx, gy, margin=1450), obs)


def trap_tail(i, local, _span):
    gx = 4200 + local * 18
    gy = -310 + (local % 9) * 78
    obs = []
    u_x = 780 + (local % 4) * 42
    u_w = 690 + (local % 5) * 34
    u_h = 640 + (local % 4) * 38
    cy = gy * 0.14

    obs.append(rect("u_left", u_x, cy, 160, u_h))
    obs.append(rect("u_top", u_x + u_w / 2, cy + u_h / 2, u_w, 160))
    obs.append(rect("u_right", u_x + u_w, cy, 160, u_h))
    obs.append(rect("inside_island", u_x + u_w * 0.54, cy - u_h * 0.16, 210, 145))

    tail_count = 5 + min(13, local // 18)
    tail_start = u_x + u_w + 740
    tail_end = gx - 420
    tail_step = (tail_end - tail_start) / max(1, tail_count - 1)
    for k in range(tail_count):
        x = tail_start + k * tail_step
        line_y = gy * (x / gx)
        sign = 1 if k == 0 else (1 if (k + local) % 2 else -1)
        y = line_y + sign * (700 + (k % 3) * 35)
        obs.append(rect(f"tail_{k+1}", x, y, 128 + (k % 3) * 20, 195 + (k % 2) * 56))
    return scenario("trap_tail", i, (gx, gy), bounds_for(gx, gy, margin=1650), obs)


def make_worlds(count=DEFAULT_COUNT):
    builders = [single_or_pair, slalom, gates, mixed_lab, trap_tail]
    counts = split_counts(count)
    scenarios = []
    difficulty = 1
    for builder, span in zip(builders, counts):
        for local in range(1, span + 1):
            scenarios.append(builder(difficulty, local, span))
            difficulty += 1
    return {
        "metadata": {
            "units": "mm",
            "count": len(scenarios),
            "description": f"{len(scenarios)} mundos progressivos para validar o controlador Bug2/Ponto-P do Khepera IV",
        },
        "scenarios": scenarios,
    }


def main():
    parser = argparse.ArgumentParser(description="Gera mundos JSON para o simulador OpenGL.")
    parser.add_argument("--count", type=int, default=DEFAULT_COUNT, help="quantidade de cenarios")
    parser.add_argument("--out", type=Path, default=OUT, help="arquivo JSON de saida")
    args = parser.parse_args()
    count = max(1, min(1000, args.count))
    args.out.write_text(json.dumps(make_worlds(count), indent=2), encoding="utf-8")
    print(f"wrote {args.out} ({count} scenarios)")


if __name__ == "__main__":
    main()
