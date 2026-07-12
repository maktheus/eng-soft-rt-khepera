"""Gera mundos JSON randomizados para o simulador OpenGL do Khepera.

O arquivo gerado e simples de editar:

{
  "scenarios": [
    {
      "name": "case_0001_single_pair",
      "difficulty": 1,
      "box_count": 1,
      "goal": [2200, 0],
      "bounds": [-450, 2700, -900, 900],
      "obstacles": [
        {"name": "box_1", "rect": [950, 0, 220, 320]}
      ]
    }
  ]
}

Todas as unidades estao em milimetros. A seed controla a randomizacao; com a
mesma seed o JSON sai identico, o que facilita depurar uma falha especifica.
"""

import argparse
import json
import random
from pathlib import Path


DEFAULT_COUNT = 1000
DEFAULT_SEED = 20260712
OUT = Path(__file__).with_name("worlds_1000.json")
FRONT_CORRIDOR_MM = 90.0


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
        "box_count": len(obstacles),
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


def jitter(rng, value, amount):
    return value + rng.uniform(-amount, amount)


def line_y(goal_x, goal_y, x):
    return goal_y * (x / goal_x)


def side_for(rng, k, bias=0):
    base = -1 if (k + bias) % 2 == 0 else 1
    return base if rng.random() > 0.28 else -base


def blocks_direct_path(goal_x, goal_y, obstacle):
    x, y, w, h = obstacle["rect"]
    if x < 420 or x > goal_x - 420:
        return False
    return abs(y - line_y(goal_x, goal_y, x)) <= h / 2.0 + FRONT_CORRIDOR_MM


def front_challenge_count(total, density, rng):
    if total <= 0:
        return 0
    count = max(1, int(round(total * density)))
    if total > count and rng.random() < 0.45:
        count += 1
    return min(total, count)


def front_random_box(goal_x, goal_y, rng, slot, total):
    x_min = 480.0
    x_max = max(x_min + 260.0, min(goal_x - 560.0, goal_x * 0.84))
    if total <= 1:
        x = rng.uniform(x_min, x_max)
    else:
        t = (slot + rng.uniform(0.15, 0.85)) / total
        x = x_min + t * (x_max - x_min)
        x = jitter(rng, x, 130.0)
    x = min(max(x, x_min), x_max)

    w = rng.uniform(110.0, 235.0)
    h = rng.uniform(170.0, 430.0)
    line = line_y(goal_x, goal_y, x)
    style = rng.random()
    if style < 0.55:
        y = line + rng.uniform(-0.25 * h, 0.25 * h)
    elif style < 0.82:
        side = rng.choice([-1, 1])
        y = line + side * rng.uniform(0.20 * h, 0.54 * h)
    else:
        y = line + rng.choice([-1, 1]) * rng.uniform(0.55 * h, 0.80 * h)

    return rect(f"front_random_{slot+1}", x, y, w, h)


def randomize_front_challenges(goal_x, goal_y, obstacles, rng, density):
    if not obstacles:
        return obstacles

    # Mantem a mesma quantidade de caixas, mas prioriza desafio frontal real:
    # varias caixas passam a cruzar ou invadir a linha A->B em posicoes
    # aleatorias. Estes cenarios sao de estresse; a chegada nao e garantida.
    count = front_challenge_count(len(obstacles), density, rng)
    indices = list(range(len(obstacles)))
    rng.shuffle(indices)
    selected = indices[:count]
    selected.sort()

    for slot, idx in enumerate(selected):
        obstacles[idx] = front_random_box(goal_x, goal_y, rng, slot, count)

    if not any(blocks_direct_path(goal_x, goal_y, obs) for obs in obstacles):
        obstacles[selected[0]] = front_random_box(goal_x, goal_y, rng, 0, 1)

    return obstacles


def single_or_pair(i, local, span, rng):
    gx = jitter(rng, 1900 + local * 18, 90)
    gy = jitter(rng, ((local % 7) - 3) * 55, 120)
    size_boost = min(90, local * 0.55)
    obs = []

    x = jitter(rng, gx * rng.uniform(0.36, 0.48), 65)
    y = line_y(gx, gy, x) + rng.choice([-1, 1]) * rng.uniform(45, 145)
    obs.append(rect(
        "box_main",
        x,
        y,
        rng.uniform(165, 225) + size_boost * 0.12,
        rng.uniform(225, 325) + size_boost * 0.18,
    ))

    if local > span * 0.45:
        x = jitter(rng, gx * rng.uniform(0.62, 0.76), 75)
        y = line_y(gx, gy, x) + rng.choice([-1, 1]) * rng.uniform(460, 650)
        obs.append(rect("box_second", x, y, rng.uniform(155, 230), rng.uniform(205, 285)))
    obs = randomize_front_challenges(gx, gy, obs, rng, density=0.90)
    return scenario("single_pair", i, (gx, gy), bounds_for(gx, gy), obs)


def slalom(i, local, _span, rng):
    count = 3 + min(9, local // 25)
    gx = jitter(rng, 2500 + local * 18, 120)
    gy = jitter(rng, 180 + ((local % 5) - 2) * 45, 150)
    amp = 275 + min(330, local * 1.25) + rng.uniform(-55, 75)
    obs = []
    start_x = rng.uniform(930, 1080)
    step = max(330, (gx - 900) / max(1, count))
    bias = rng.randint(0, 1)
    for k in range(count):
        x = jitter(rng, start_x + k * step, min(95, step * 0.22))
        side = side_for(rng, k, bias)
        y = line_y(gx, gy, x) + side * rng.uniform(amp, amp + 170)
        obs.append(rect(
            f"slalom_{k+1}",
            x,
            y,
            rng.uniform(140, 215),
            rng.uniform(245, 395),
        ))
    obs = randomize_front_challenges(gx, gy, obs, rng, density=0.42)
    return scenario(f"slalom_{count}", i, (gx, gy), bounds_for(gx, gy, margin=1100), obs)


def gates(i, local, _span, rng):
    gate_count = 2 + min(7, local // 28)
    gx = jitter(rng, 3000 + local * 17, 130)
    gy = jitter(rng, ((local % 7) - 3) * 50, 120)
    gap = max(430, 650 - local * 0.72 + rng.uniform(-34, 42))
    obs = []
    gate_start = rng.uniform(1030, 1180)
    step = max(560, (gx - gate_start - 760) / max(1, gate_count - 1))
    for k in range(gate_count):
        x = jitter(rng, gate_start + k * step, min(95, step * 0.18))
        lane_y = line_y(gx, gy, x) + rng.uniform(-70, 70)
        height = rng.uniform(435, 650)
        width = rng.uniform(170, 215)
        obs.append(rect(f"gate_{k+1}_top", x, lane_y + gap / 2 + height / 2, width, height))
        obs.append(rect(f"gate_{k+1}_bottom", x, lane_y - gap / 2 - height / 2, width, height))
        if k % 2 == 1 and local > 40:
            ox = x + rng.uniform(260, 380)
            oy = line_y(gx, gy, ox) + rng.choice([-1, 1]) * rng.uniform(300, 500)
            obs.append(rect(f"offset_{k+1}", ox, oy, rng.uniform(145, 190), rng.uniform(210, 295)))
    obs = randomize_front_challenges(gx, gy, obs, rng, density=0.50)
    return scenario(f"gates_gap_{int(gap)}", i, (gx, gy), bounds_for(gx, gy, margin=1250), obs)


def mixed_lab(i, local, _span, rng):
    gx = jitter(rng, 3400 + local * 16, 160)
    gy = jitter(rng, 520 + ((local % 8) - 3) * 65, 170)
    count = 8 + min(28, local // 10)
    obs = []
    bias = rng.randint(0, 1)
    for k in range(count):
        nominal_x = 520 + k * ((gx - 1120) / max(1, count - 1))
        x = jitter(rng, nominal_x, 105)
        corridor = rng.uniform(760, 1080)
        side = side_for(rng, k, bias)
        if rng.random() < 0.18:
            # Algumas caixas ficam bem longe da linha para ampliar o mundo sem
            # fechar a passagem principal.
            side = rng.choice([-1, 1])
            corridor = rng.uniform(1180, 1500)
        y = line_y(gx, gy, x) + side * corridor + rng.uniform(-95, 95)
        obs.append(rect(
            f"lab_{k+1}",
            x,
            y,
            rng.uniform(125, 215),
            rng.uniform(155, 300),
        ))
    obs = randomize_front_challenges(gx, gy, obs, rng, density=0.34)
    return scenario(f"mixed_lab_{count}", i, (gx, gy), bounds_for(gx, gy, margin=1650), obs)


def trap_tail(i, local, _span, rng):
    gx = jitter(rng, 4200 + local * 18, 190)
    gy = jitter(rng, -310 + (local % 9) * 78, 145)
    obs = []

    tail_count = 5 + min(13, local // 18)
    bias = rng.randint(0, 1)

    # Quatro caixas iniciais substituem o U fixo: continuam formando uma
    # armadilha perto da linha, mas sem criar cavidade fechada que prenda Bug2.
    for k in range(4):
        x = jitter(rng, 720 + k * rng.uniform(300, 390), 70)
        if k == 0:
            offset = rng.choice([-1, 1]) * rng.uniform(120, 220)
        else:
            offset = side_for(rng, k, bias) * rng.uniform(520, 840)
        y = line_y(gx, gy, x) + offset
        obs.append(rect(
            f"trap_{k+1}",
            x,
            y,
            rng.uniform(135, 230),
            rng.uniform(190, 360),
        ))

    tail_start = 2050 + rng.uniform(-80, 120)
    tail_end = gx - rng.uniform(380, 500)
    tail_step = (tail_end - tail_start) / max(1, tail_count - 1)
    for k in range(tail_count):
        x = jitter(rng, tail_start + k * tail_step, min(105, tail_step * 0.20))
        side = side_for(rng, k + 4, bias)
        y = line_y(gx, gy, x) + side * rng.uniform(690, 940)
        obs.append(rect(
            f"tail_{k+1}",
            x,
            y,
            rng.uniform(118, 180),
            rng.uniform(185, 280),
        ))
    obs = randomize_front_challenges(gx, gy, obs, rng, density=0.30)
    return scenario("trap_tail", i, (gx, gy), bounds_for(gx, gy, margin=1750), obs)


def make_worlds(count=DEFAULT_COUNT, seed=DEFAULT_SEED):
    builders = [single_or_pair, slalom, gates, mixed_lab, trap_tail]
    counts = split_counts(count)
    scenarios = []
    difficulty = 1
    for family, (builder, span) in enumerate(zip(builders, counts)):
        for local in range(1, span + 1):
            scenario_rng = random.Random(seed + difficulty * 1009 + family * 9176)
            scenarios.append(builder(difficulty, local, span, scenario_rng))
            difficulty += 1
    return {
        "metadata": {
            "units": "mm",
            "count": len(scenarios),
            "seed": seed,
            "description": (
                f"{len(scenarios)} mundos randomizados progressivos de estresse, "
                "com caixas variadas no corredor frontal do Khepera IV"
            ),
        },
        "scenarios": scenarios,
    }


def main():
    parser = argparse.ArgumentParser(description="Gera mundos JSON randomizados para o simulador OpenGL.")
    parser.add_argument("--count", type=int, default=DEFAULT_COUNT, help="quantidade de cenarios")
    parser.add_argument("--seed", type=int, default=DEFAULT_SEED, help="seed da randomizacao")
    parser.add_argument("--out", type=Path, default=OUT, help="arquivo JSON de saida")
    args = parser.parse_args()
    count = max(1, min(1000, args.count))
    args.out.write_text(json.dumps(make_worlds(count, args.seed), indent=2), encoding="utf-8")
    print(f"wrote {args.out} ({count} scenarios, seed={args.seed})")


if __name__ == "__main__":
    main()
