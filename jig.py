"""QDD トルク測定治具（キッチンスケール＋レバーアーム方式）

座標系（組立状態）:
    X: 形状は +X 側にアームを伸ばして作り、ARM_SIDE="right" のとき最後に X を反転する
       （出力側からモーター正面を見て、-X が右）
    Y: モーター軸方向。立板の前面が Y=0。モーターは前面（出力側）の外周を立板の背面に
       ボルト留めし、本体は -Y 側に張り出す。出力部は立板中央の穴の中にあり、
       アームはその穴越しに出力フランジへ取り付けて立板の前面側（+Y）に出る
    Z: 鉛直上向き（ベース底面が Z=0）

モーター出力に付けたアーム先端の接触突起が、ベースの横に置いたキッチンスケールの
皿を上から押す。トルク T = F * ARM_RADIUS（F = 表示値 [kg] x 9.81）。
先端の穴は校正用の錘を吊るすためのもので、接触突起の真上（同じ半径）にある。

`uv run python jig.py` で out/ に STL（印刷向きに配置済み）と組立確認用 STEP を出力する。
"""

from math import acos, atan2, cos, hypot, sin
from pathlib import Path

from build123d import (
    Align,
    Box,
    Circle,
    Compound,
    Cylinder,
    Location,
    Part,
    Plane,
    Polygon,
    Polyline,
    Rectangle,
    Rot,
    Pos,
    extrude,
    export_step,
    export_stl,
    make_face,
    mirror,
)

# ---------------------------------------------------------------------------
# モーター寸法（実測）
# ---------------------------------------------------------------------------
MOTOR_OD = 70.0  # モーター本体外径
MOTOR_BODY_LEN = 40.0  # 本体の軸方向長さ（仮。組立確認用のダミーにのみ使う）
OUT_PROTRUDE = 2.0  # 固定側取付面（前面外周）から出力フランジ面までの突出量
STATOR_PCD = 64.0  # 固定側取付穴の PCD（向かい合う穴の中心間距離）
STATOR_N = 4  # 固定側取付穴の数（90° 間隔）
STATOR_ANGLE_OFFSET = 45.0  # 固定側取付穴の位相 [deg]（45 で斜め × 配置）
STATOR_HOLE = 3.4  # 固定側取付穴径（M3）
STATOR_BORE = 52.0  # 立板中央の逃げ穴径（出力部とハブが入る）
OUT_FLANGE_D = 45.0  # 出力フランジ外径（実測）
OUT_PCD = 36.0  # 出力側取付穴の PCD（実測: 中心から 18 mm）
OUT_N = 6  # 出力側取付穴の数（実測）
OUT_HOLE = 3.4  # 出力側取付穴径（M3）
OUT_CBORE_D = 6.5  # 出力側ボルト頭の座ぐり径
OUT_CBORE_DEPTH = 3.5
OUT_ANGLE_OFFSET = 0.0  # 出力側取付穴の位相 [deg]（0 でアーム方向に 1 穴目）

# ---------------------------------------------------------------------------
# キッチンスケール（ベースの横の机に置く）
# ---------------------------------------------------------------------------
SCALE_H = 33.0  # 机から皿の上面までの高さ（実測）。突起の高さより低い分は下に板などを敷く
SCALE_W = 150.0  # 皿の幅（組立確認用のダミーにのみ使う）
SCALE_GAP = 2.0  # ベースとスケールの隙間

# ---------------------------------------------------------------------------
# 治具寸法
# ---------------------------------------------------------------------------
ARM_RADIUS = 100.0  # モーター軸から接触突起の先端までの水平距離（トルク換算に使う）
HUB_D = 45.0  # ハブ外径
HUB_T = 15.0  # ハブ厚（Y 方向）
ARM_T = 10.0  # アーム厚（Y 方向）。外側の面をハブの外側の面と揃え、モーター側を浮かせる
ARM_H = 24.0  # 根本のアーム高さ（Z 方向、曲げ強度を決める）
ARM_TIP_H = ARM_H * 0.5  # 先端（接触突起の位置）のアーム高さ。根本から直線テーパー
PROBE_A = 5.0  # 接触突起（三角形）の底辺の半幅（X 方向）
PROBE_TIP_R = 1.5  # 接触突起の頂点の丸み半径
TIP_HOLE_D = 5.0  # 校正用の錘を吊るす穴。突起の真上、アーム先端の高さ中央を Y 方向に貫通
ARM_SIDE = "right"  # 出力側からモーター正面を見たときにアームが伸びる側（"right" / "left"）
# 突起の先端（最下点）はハブ下端の接線（Z = AXIS_Z - HUB_D/2）上に置く。
# 三角形の高さは先端下面からその接線までの距離で決まる

BASE_T = 8.0  # ベース板厚
PLATE_T = 5.0  # 立板厚。モーター固定ボルトの頭を沈めてもアームと隙間が残る厚さ
STATOR_CBORE_D = 6.5  # 立板前面のボルト頭座ぐり径（M3）
STATOR_CBORE_DEPTH = PLATE_T - 2.0  # 頭の下に 2 mm 残す（M3 キャップボルトの頭は 3 mm で面一）
MOTOR_CLEAR = 5.0  # モーター外周とベース上面の隙間
GUSSET_T = 8.0
GUSSET_DEPTH = 20.0
GUSSET_H = 60.0
BASE_MARGIN_X = 25.0  # アームと反対側の余白（クランプ用）
BASE_MARGIN_ARM = 3.0  # アーム側の余白（スケールを近づけるため最小限）
BASE_FRONT = 20.0  # 立板の前面からベース前端まで
BASE_HOLE = 5.5  # ベース四隅の固定穴（M5 ねじ止め用、クランプのみなら不使用）

# ---------------------------------------------------------------------------
# 派生寸法
# ---------------------------------------------------------------------------
# モーター軸の高さ: モーターがベースに当たらず、突起がスケールの皿に届く高さ
AXIS_Z = max(BASE_T + MOTOR_OD / 2 + MOTOR_CLEAR, SCALE_H + HUB_D / 2)
FLANGE_Y = -PLATE_T + OUT_PROTRUDE  # 出力フランジ面の Y（立板の穴の中）
ARM_Y = FLANGE_Y + HUB_T - ARM_T / 2  # アーム中心面の Y
ARM_INNER_Y = FLANGE_Y + HUB_T - ARM_T  # アームの立板側の面
PLATE_W = MOTOR_OD + 24.0
PLATE_TOP = AXIS_Z + MOTOR_OD / 2 + 10.0

PROBE_Z = AXIS_Z - HUB_D / 2  # 突起の最下点の高さ（机から）＝ハブ下端の接線

BASE_X0 = -PLATE_W / 2 - BASE_MARGIN_X
BASE_X1 = PLATE_W / 2 + BASE_MARGIN_ARM
BASE_Y0 = -PLATE_T - GUSSET_DEPTH - 6.0
BASE_Y1 = BASE_FRONT

assert HUB_D / 2 > ARM_TIP_H / 2, "接触突起が下に伸びない: HUB_D か ARM_TIP_H を見直す"
BED = 180.0  # 造形エリア
assert BASE_X1 - BASE_X0 <= BED and BASE_Y1 - BASE_Y0 <= BED, "ベースが造形エリアを超える"
assert HUB_D / 2 + ARM_RADIUS + ARM_TIP_H / 2 <= BED, "アームが造形エリアを超える"
assert ARM_RADIUS > BASE_X1 + SCALE_GAP + 10.0, "突起がスケールの皿の端に近すぎる"
assert ARM_INNER_Y >= 1.0, "アームが立板の前面に近すぎる: HUB_T / ARM_T / PLATE_T を見直す"
assert STATOR_BORE > HUB_D + 2.0, "ハブが立板中央の穴に入らない"
assert PLATE_W / 2 - GUSSET_T > MOTOR_OD / 2 + 1.0, "背面のリブがモーター本体と干渉する"


def bolt_circle(pcd: float, n: int, offset_deg: float) -> list[tuple[float, float]]:
    """PCD 上の穴位置（極座標 → 2D）。"""
    from math import cos, radians, sin

    return [
        (
            pcd / 2 * cos(radians(offset_deg + 360 / n * i)),
            pcd / 2 * sin(radians(offset_deg + 360 / n * i)),
        )
        for i in range(n)
    ]


def make_base() -> Part:
    # ベース板
    base = Pos(BASE_X0, BASE_Y0, 0) * Box(
        BASE_X1 - BASE_X0,
        BASE_Y1 - BASE_Y0,
        BASE_T,
        align=(Align.MIN, Align.MIN, Align.MIN),
    )

    # 立板（Y=-PLATE_T..0）
    plate = Pos(0, -PLATE_T, 0) * Box(
        PLATE_W, PLATE_T, PLATE_TOP, align=(Align.CENTER, Align.MIN, Align.MIN)
    )
    base += plate

    # 背面のリブ（YZ 平面の三角形を X 方向に押し出し）
    rib_profile = make_face(
        Polyline(
            (-PLATE_T, BASE_T),
            (-PLATE_T, BASE_T + GUSSET_H),
            (-PLATE_T - GUSSET_DEPTH, BASE_T),
            close=True,
        )
    )
    for x in (-PLATE_W / 2 + GUSSET_T / 2, PLATE_W / 2 - GUSSET_T / 2):
        rib = extrude(Plane.YZ.offset(x - GUSSET_T / 2) * rib_profile, GUSSET_T)
        base += rib

    # 立板の穴: 中央逃げ穴 + 固定側取付穴（軸は Y 方向）。
    # ボルトは前面から通してモーター前面のネジ穴へ。頭は前面の座ぐりに沈める
    to_axis = Pos(0, 0, AXIS_Z) * Rot(90, 0, 0)  # Z 軸の円柱を Y 軸方向へ（局所 Z → -Y）
    base -= to_axis * Cylinder(STATOR_BORE / 2, 100)
    for x, z in bolt_circle(STATOR_PCD, STATOR_N, STATOR_ANGLE_OFFSET):
        base -= to_axis * Pos(x, z, 0) * Cylinder(STATOR_HOLE / 2, 100)
        base -= to_axis * Pos(x, z, 0) * Cylinder(
            STATOR_CBORE_D / 2, STATOR_CBORE_DEPTH, align=(Align.CENTER, Align.CENTER, Align.MIN)
        )

    # 四隅の固定穴
    m = 7.0
    rib_x = PLATE_W / 2 - GUSSET_T / 2
    for x in (BASE_X0 + m, BASE_X1 - m):
        for y in (BASE_Y0 + m, BASE_Y1 - m):
            if y < -PLATE_T and abs(abs(x) - rib_x) < GUSSET_T / 2 + BASE_HOLE / 2 + 1.0:
                continue  # リブと重なる位置には開けない
            base -= Pos(x, y, 0) * Cylinder(
                BASE_HOLE / 2, 50, align=(Align.CENTER, Align.CENTER, Align.MIN)
            )

    return base


def make_arm() -> Part:
    """組立座標で作る（出力フランジ面 Y=FLANGE_Y に密着）。"""
    to_axis = Pos(0, FLANGE_Y, AXIS_Z) * Rot(-90, 0, 0)  # 局所 Z → +Y
    outer_y = FLANGE_Y + HUB_T  # ハブとアームの外側の面

    # ハブ（Y = FLANGE_Y..outer_y）。立板中央の穴に入ってフランジに当たる
    arm = to_axis * Cylinder(HUB_D / 2, HUB_T, align=(Align.CENTER, Align.CENTER, Align.MIN))

    # アームの XZ 外形（原点はモーター軸）: テーパー梁 + 先端の半円 + 接触突起
    profile = Polygon(
        (0, -ARM_H / 2),
        (ARM_RADIUS, -ARM_TIP_H / 2),
        (ARM_RADIUS, ARM_TIP_H / 2),
        (0, ARM_H / 2),
        align=None,
    )
    profile += Pos(ARM_RADIUS, 0) * Circle(ARM_TIP_H / 2)
    # 接触突起: 先端下面から下へ伸びる三角形。頂点は半径 PROBE_TIP_R で丸め、
    # 丸めた最下点がハブ下端の接線に接する
    tip_bottom = -ARM_TIP_H / 2
    c = (ARM_RADIUS, -HUB_D / 2 + PROBE_TIP_R)  # 頂点の丸みの中心
    # 底辺の右端から丸みの円への接点（左側は X 対称）
    dx, dz = PROBE_A, tip_bottom - c[1]
    t = atan2(dz, dx) - acos(PROBE_TIP_R / hypot(dx, dz))
    tx, tz = PROBE_TIP_R * cos(t), PROBE_TIP_R * sin(t)
    profile += Polygon(
        (ARM_RADIUS - PROBE_A, tip_bottom),
        (c[0] - tx, c[1] + tz),
        (c[0] + tx, c[1] + tz),
        (ARM_RADIUS + PROBE_A, tip_bottom),
        align=None,
    )
    profile += Pos(*c) * Circle(PROBE_TIP_R)
    # 三角形の上端と先端の半円の間の隙間を埋める
    profile += Pos(ARM_RADIUS, tip_bottom) * Rectangle(
        2 * PROBE_A, ARM_TIP_H / 2, align=(Align.CENTER, Align.MIN)
    )

    # 外側の面に置いて -Y 方向へアーム厚ぶん押し出す（Y = outer_y - ARM_T..outer_y）
    plane = Plane(origin=(0, outer_y, AXIS_Z), x_dir=(1, 0, 0), z_dir=(0, -1, 0))
    arm += extrude(plane * profile, ARM_T, dir=(0, -1, 0))

    # 出力側取付穴（ボルト頭は外側＝+Y 側に座ぐり）
    for x, z in bolt_circle(OUT_PCD, OUT_N, OUT_ANGLE_OFFSET):
        # to_axis の局所 Y は -Z に対応するので z の符号を反転
        p = to_axis * Pos(x, -z, 0)
        arm -= p * Cylinder(OUT_HOLE / 2, 100)
        arm -= p * Pos(0, 0, HUB_T - OUT_CBORE_DEPTH) * Cylinder(
            OUT_CBORE_D / 2, OUT_CBORE_DEPTH + 1, align=(Align.CENTER, Align.CENTER, Align.MIN)
        )

    # 突起の真上（軸から ARM_RADIUS、アーム高さの中央）の貫通穴
    arm -= Pos(ARM_RADIUS, 0, AXIS_Z) * Rot(90, 0, 0) * Cylinder(TIP_HOLE_D / 2, 200)
    return arm


def make_scale_dummy() -> Part:
    """組立確認用のキッチンスケール（皿の上面を突起の高さに合わせた状態）。"""
    return Pos(BASE_X1 + SCALE_GAP, ARM_Y, 0) * Box(
        SCALE_W, SCALE_W, PROBE_Z, align=(Align.MIN, Align.CENTER, Align.MIN)
    )


def make_motor_dummy() -> Part:
    """組立確認用のモーター外形（本体は立板の背面側、出力部は立板の穴の中）。"""
    to_axis = Pos(0, -PLATE_T, AXIS_Z) * Rot(-90, 0, 0)  # 局所 Z → +Y
    body = to_axis * Cylinder(
        MOTOR_OD / 2, MOTOR_BODY_LEN, align=(Align.CENTER, Align.CENTER, Align.MAX)
    )
    output = to_axis * Cylinder(
        OUT_FLANGE_D / 2, OUT_PROTRUDE, align=(Align.CENTER, Align.CENTER, Align.MIN)
    )
    return body + output


def to_print_pose_arm(arm: Part) -> Part:
    """ハブとアームが揃った外側の面（+Y 面）を下にして寝かせ、Z=0 に置く。
    曲げ方向の応力が積層面に沿うため強度上有利。"""
    a = Rot(-90, 0, 0) * arm  # +Y 面が下
    bb = a.bounding_box()
    return Pos(-bb.min.X, -bb.min.Y, -bb.min.Z) * a


def handed(part: Part) -> Part:
    """ARM_SIDE に合わせて左右を決める（形状は +X 側＝正面から見て左で作っている）。"""
    return mirror(part, Plane.YZ) if ARM_SIDE == "right" else part


def main() -> None:
    out = Path(__file__).parent / "out"
    out.mkdir(exist_ok=True)
    base_stl, arm_stl, asm_step = out / "base.stl", out / "arm.stl", out / "assembly.step"
    # ビューアーが古い内容を表示し続けないよう、既存の出力を先に削除する
    for f in (base_stl, arm_stl, asm_step):
        f.unlink(missing_ok=True)

    base = handed(make_base())
    arm = handed(make_arm())

    for name, part in (("base", base), ("arm", arm)):
        assert part.is_valid, f"{name} is invalid"
        assert len(part.solids()) == 1, f"{name} が 1 つの立体になっていない"
        bb = part.bounding_box()
        print(f"{name:5s} size = {bb.size.X:6.1f} x {bb.size.Y:6.1f} x {bb.size.Z:6.1f} mm, "
              f"volume = {part.volume / 1000:6.1f} cm^3")

    # 干渉チェック（アーム・ベース・モーター・スケールのダミー）
    sc = handed(make_scale_dummy())
    motor = make_motor_dummy()
    for a_name, a, b_name, b in (
        ("arm", arm, "base", base),
        ("arm", arm, "scale", sc),
        ("motor", motor, "base", base),
        ("motor", motor, "scale", sc),
        ("base", base, "scale", sc),
    ):
        v = (a & b).volume
        print(f"interference {a_name}-{b_name}: {v:.3f} mm^3")
        assert v < 0.01, f"{a_name} と {b_name} が干渉している"

    export_stl(base, base_stl)
    export_stl(to_print_pose_arm(arm), arm_stl)

    base.label, arm.label, sc.label, motor.label = "base", "arm", "scale", "motor"
    export_step(Compound([base, arm, sc, motor]), asm_step)

    print(f"axis height = {AXIS_Z:.1f} mm, probe height = {PROBE_Z:.1f} mm, arm radius = {ARM_RADIUS:.1f} mm")
    print(f"T [N·m] = F [N] x {ARM_RADIUS / 1000:.4f}")


if __name__ == "__main__":
    main()
