"""README 用の寸法図（docs/arm.png, docs/base.png）を jig.py の現在値から作る。

    uv run --with matplotlib python docs/make_figures.py
"""

import sys
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
import jig  # noqa: E402

plt.rcParams["font.family"] = ["Hiragino Sans", "sans-serif"]
DOCS = Path(__file__).parent


def draw(ax, part, h, v, color, sign_h=1.0):
    """部品を (h, v) 軸の平面に投影して塗る。"""
    verts, tris = part.tessellate(0.2)
    for t in tris:
        ax.fill(
            [sign_h * getattr(verts[i], h) for i in t],
            [getattr(verts[i], v) for i in t],
            color=color,
            lw=0,
        )


def dim(ax, p0, p1, text, off=(0, 0), **kw):
    """寸法線（両矢印）と寸法値。"""
    ax.annotate("", p0, p1, arrowprops=dict(arrowstyle="<->", lw=0.8, color="k"))
    mx, my = (p0[0] + p1[0]) / 2 + off[0], (p0[1] + p1[1]) / 2 + off[1]
    ax.text(mx, my, text, ha="center", va="center", fontsize=9,
            bbox=dict(fc="white", ec="none", pad=1), **kw)


def finish(ax, title, pad=30):
    # 寸法線は描画範囲の計算に入らないので、余白を広げて矢印が切れないようにする
    x0, x1 = ax.get_xlim()
    y0, y1 = ax.get_ylim()
    sx = 1 if x1 > x0 else -1
    sy = 1 if y1 > y0 else -1
    ax.set_xlim(x0 - sx * pad, x1 + sx * pad)
    ax.set_ylim(y0 - sy * pad, y1 + sy * pad)
    ax.set_aspect("equal")
    ax.set_title(title, fontsize=11)
    ax.axis("off")


def arm_figure():
    arm = jig.handed(jig.make_arm())
    z0 = jig.AXIS_Z
    R, y_in, y_out = jig.ARM_RADIUS, jig.FLANGE_Y, jig.FLANGE_Y + jig.HUB_T
    fig, (a1, a2) = plt.subplots(2, 1, figsize=(9, 8), gridspec_kw=dict(height_ratios=[3, 1.4]))

    # 正面（出力側から見る）: 画面右 = -X
    draw(a1, arm, "X", "Z", "#e08a4a", sign_h=-1)
    a1.plot([0], [z0], "k+", ms=10)
    dim(a1, (0, z0 + 30), (R, z0 + 30), f"{R:.0f}（軸〜突起先端）")
    a1.plot([0, 0], [z0, z0 + 32], "k:", lw=0.6)
    a1.plot([R, R], [jig.PROBE_Z, z0 + 32], "k:", lw=0.6)
    dim(a1, (-jig.HUB_D / 2, z0 - 30), (jig.HUB_D / 2, z0 - 30), f"ハブ φ{jig.HUB_D:.0f}")
    dim(a1, (-34, z0 - jig.ARM_H / 2), (-34, z0 + jig.ARM_H / 2), f"{jig.ARM_H:.0f}", off=(-6, 0))
    a1.plot([-34, 0], [z0 + jig.ARM_H / 2] * 2, "k:", lw=0.6)
    a1.plot([-34, 0], [z0 - jig.ARM_H / 2] * 2, "k:", lw=0.6)
    dim(a1, (R + 14, z0 - jig.ARM_TIP_H / 2), (R + 14, z0 + jig.ARM_TIP_H / 2),
        f"{jig.ARM_TIP_H:.0f}", off=(6, 0))
    a1.plot([-60, R + 20], [jig.PROBE_Z] * 2, "r--", lw=0.6)
    a1.text(R + 22, jig.PROBE_Z, "ハブ下端の接線＝突起の最下点", color="r", fontsize=8, va="center")
    a1.text(R - 2, z0 + 9, f"校正用の穴 φ{jig.TIP_HOLE_D:.0f}", fontsize=8, ha="right")
    a1.text(0, z0 - 40,
            f"M3 用 φ{jig.OUT_HOLE} × {jig.OUT_N}（PCD {jig.OUT_PCD:.0f}）", fontsize=8, ha="center")
    finish(a1, "アーム：正面（出力側から見る）")

    # 上面: 横 = 正面から見た左右、縦 = Y（上がモーター側）
    draw(a2, arm, "X", "Y", "#e08a4a", sign_h=-1)
    a2.invert_yaxis()
    a2.axhline(0, color="#7aa6d8", lw=1.5)
    a2.text(R / 2, -1.5, "立板の前面（アームとの隙間 2）", color="#4a76a8", fontsize=8, ha="center", va="bottom")
    dim(a2, (-30, y_in), (-30, y_out), f"ハブ {jig.HUB_T:.0f}", off=(-14, 0))
    dim(a2, (60, y_out - jig.ARM_T), (60, y_out), f"アーム {jig.ARM_T:.0f}", off=(14, 0))
    a2.text(R + 12, y_in, "← 出力フランジ面（ハブが当たる面）", fontsize=8, va="center")
    finish(a2, "アーム：上面（下が手前）")
    fig.tight_layout()
    fig.savefig(DOCS / "arm.png", dpi=110)


def base_figure():
    base = jig.handed(jig.make_base())
    bb = base.bounding_box()
    z0 = jig.AXIS_Z
    fig, (a1, a2) = plt.subplots(1, 2, figsize=(11, 5), gridspec_kw=dict(width_ratios=[1.6, 1]))

    # 正面（出力側から見る）
    draw(a1, base, "X", "Z", "#7aa6d8", sign_h=-1)
    left, right = -bb.max.X, -bb.min.X
    dim(a1, (left, -8), (right, -8), f"{bb.size.X:.0f}")
    dim(a1, (right + 8, 0), (right + 8, bb.max.Z), f"{bb.max.Z:.1f}", off=(8, 0))
    dim(a1, (right + 22, 0), (right + 22, z0), f"軸 {z0:.1f}", off=(10, 0))
    a1.plot([0, right + 24], [z0, z0], "k:", lw=0.6)
    dim(a1, (-jig.STATOR_BORE / 2, z0), (jig.STATOR_BORE / 2, z0), f"φ{jig.STATOR_BORE:.0f}")
    a1.text(0, z0 + jig.STATOR_PCD / 2 + 4,
            f"M3 用 φ{jig.STATOR_HOLE} × {jig.STATOR_N}（PCD {jig.STATOR_PCD:.0f}、45°）",
            fontsize=8, ha="center")
    dim(a1, (left, jig.BASE_T + 4), (left + jig.BASE_MARGIN_X, jig.BASE_T + 4),
        f"{jig.BASE_MARGIN_X:.0f}", off=(0, 5))
    a1.text(left, jig.BASE_T + 16, "クランプ用", fontsize=8)
    a1.text(right, jig.BASE_T + 4, " → スケール側", fontsize=8, va="bottom")
    finish(a1, "台座：正面（出力側から見る）")

    # 側面: 横 = Y（右が手前）
    draw(a2, base, "Y", "Z", "#7aa6d8")
    dim(a2, (bb.min.Y, -8), (bb.max.Y, -8), f"{bb.size.Y:.0f}")
    dim(a2, (-jig.PLATE_T, bb.max.Z + 6), (0, bb.max.Z + 6), f"立板 {jig.PLATE_T:.0f}", off=(14, 0))
    dim(a2, (bb.max.Y + 6, 0), (bb.max.Y + 6, jig.BASE_T), f"{jig.BASE_T:.0f}", off=(6, 0))
    a2.text(bb.min.Y, jig.BASE_T + jig.GUSSET_H + 3, "背面のリブ", fontsize=8)
    a2.text(bb.max.Y, jig.BASE_T + 6, "手前（アーム側）→", fontsize=8, ha="right")
    finish(a2, "台座：側面")
    fig.tight_layout()
    fig.savefig(DOCS / "base.png", dpi=110)


if __name__ == "__main__":
    arm_figure()
    base_figure()
