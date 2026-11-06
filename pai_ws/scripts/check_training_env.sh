#!/usr/bin/env bash
# 学習環境の点検: 設定の整合・姿勢ファイル・記録済み bag の中身をまとめて確認する。
# 使い方: bash check_training_env.sh      （Gazebo 起動中なら /cube_pose の有無も確認）
TRAIN="${TRAIN:-$HOME/pai_training}"
WS="${WS:-$HOME/pai_ws}"
export TRAIN WS
python3 - <<'EOF'
import os, re, glob, sqlite3, math
T, W = os.environ['TRAIN'], os.environ['WS']
ng = 0
def ok(m): print('OK  ', m)
def bad(m):
    global ng; ng += 1; print('NG  ', m)
def info(m): print('INFO', m)
def yaml_get(path, key):
    for line in open(path, encoding='utf-8'):
        m = re.match(r'\s*%s\s*:\s*"?([^"#\n]*?)"?\s*(#.*)?$' % key, line)
        if m: return m.group(1).strip()
def read(p): return open(p, encoding='utf-8').read()

print('--- 設定ファイル')
conv, train = f'{T}/configs/conversion_config.yaml', f'{T}/configs/train_config.yaml'
for f in (conv, train):
    ok(f) if os.path.exists(f) else bad(f'{f} がありません')
feat_rel = yaml_get(conv, 'config') if os.path.exists(conv) else None
feat_t = yaml_get(train, 'config_path') if os.path.exists(train) else None
if feat_rel != feat_t: bad(f'変換({feat_rel})と学習({feat_t})で features ファイルが違います')
else: ok(f'features ファイル共通: {feat_rel}')
if feat_rel and os.path.exists(f'{T}/{feat_rel}'):
    src = read(f'{T}/{feat_rel}')
    m = re.search(r'"observation\.environment_state".*?"shape":\s*\((\d+),\)', src, re.S)
    if m and m.group(1) == '2': ok('environment_state は 2 次元（立方体 x, y）')
    else: bad(f'environment_state の shape が 2 ではありません（{m.group(1) if m else "不明"}）。crane_features_push.py を使うこと')
else:
    bad(f'features ファイルがありません: {feat_rel}')
out_c, root_t = yaml_get(conv, 'output_dir'), yaml_get(train, 'dataset_root')
ok(f'出力先と学習データが一致: {out_c}') if out_c == root_t else bad(f'変換 output_dir({out_c}) と学習 dataset_root({root_t}) が違います')
info(f"学習設定: steps={yaml_get(train,'training_steps')} batch={yaml_get(train,'batch_size')} device={yaml_get(train,'device')}"
     "（200/2 は動作確認用。本番は configs/train_config.yaml.push の値）")

print('--- スクリプト')
inf = read(f'{T}/scripts/infer.py')
ok('infer.py は /cube_pose を使う') if 'latest_cube' in inf and "'/cube_pose'" in inf else bad('infer.py が未パッチ（環境状態が関節状態の複製のまま）')
py = read(f'{T}/pyproject.toml')
ok('pixi task convert-push あり') if 'convert-push' in py else bad('pyproject.toml に convert-push がありません')
for f in ('push_demo.py', 'cube_pose_relay.py', 'reset_cube.sh', 'episode.sh', 'rosbag2lerobot_push.py'):
    ok(f) if os.path.exists(f'{T}/scripts/{f}') else bad(f'scripts/{f} がありません')
crlf = [p for p in glob.glob(f'{T}/scripts/*') + glob.glob(f'{T}/configs/*') + glob.glob(f'{W}/scripts/*')
        if os.path.isfile(p) and b'\r' in open(p, 'rb').read()]
ok('CRLF なし') if not crlf else bad('CRLF を含むファイル: ' + ', '.join(crlf))

print('--- 姿勢ファイル')
LIM = [(-2.6179, 2.6179), (-1.8247, 1.7837), (-2.5565, 2.4542), (-1.8400, 1.7683)]
for n in ('startp0.txt', 'startp.txt'):
    p = f'{W}/scripts/{n}'
    if not os.path.exists(p): bad(f'{p} がありません'); continue
    v = {m.group(1): float(m.group(2)) for m in
         (re.match(r'\s*(crane_plus_\w+):\s*([+-]?[\d.]+)\s*rad', l) for l in open(p, encoding='utf-8')) if m}
    q = [v.get(f'crane_plus_joint{i}') for i in range(1, 5)]
    if None in q: bad(f'{n}: 関節の行が足りません'); continue
    out = [i + 1 for i, (x, (lo, hi)) in enumerate(zip(q, LIM)) if not lo <= x <= hi]
    ok(f'{n}: ' + ' '.join(f'{x:+.2f}' for x in q)) if not out else bad(f'{n}: joint{out} が可動範囲外')

print('--- 記録済み bag (dataset/rosbags/push)')
need = ['/joint_states', '/crane_plus_arm_controller/joint_trajectory',
        '/crane_plus_gripper_controller/joint_trajectory', '/cube_pose']
bagdir = yaml_get(conv, 'bag_dir') or os.path.expanduser('~/dataset/rosbags/push')
dbs = sorted(glob.glob(f'{bagdir}/**/*.db3', recursive=True))
if not dbs: info(f'bag がまだありません: {bagdir}')
for db in dbs:
    try:
        c = sqlite3.connect(f'file:{db}?mode=ro', uri=True)
        cnt = dict(c.execute('select t.name, count(m.id) from topics t left join messages m on m.topic_id=t.id group by t.name').fetchall())
        miss = [t for t in need if cnt.get(t, 0) == 0]
        name = os.path.basename(os.path.dirname(db))
        ok(f'{name}: ' + ' '.join(f'{cnt[t]}' for t in need)) if not miss else bad(f'{name}: 欠落/0件 -> {miss}')
    except Exception as e:
        bad(f'{db}: {e}')
if dbs: info(f'{len(dbs)} エピソード（各トピックの件数: joint_states, arm, gripper, cube_pose の順）')

print('--- 実行環境')
import shutil
ok('pixi あり') if shutil.which('pixi') else bad('pixi が見つかりません')
ok('pixi 環境 (.pixi) あり') if os.path.isdir(f'{T}/.pixi') else bad('pixi install が未実行です')
ds = out_c
info(f'データセット出力先 {"あり（再変換時は上書き確認が出ます）" if ds and os.path.exists(ds) else "なし（未変換）"}: {ds}')
print()
print('結果:', 'すべて OK' if ng == 0 else f'NG {ng} 件')
EOF
echo
echo "--- /cube_pose（Gazebo 起動中のみ）"
if command -v ros2 >/dev/null 2>&1 || source /opt/ros/humble/setup.bash 2>/dev/null; then
  source ~/pai_ws/install/setup.bash 2>/dev/null
  if timeout 5 ros2 topic echo /cube_pose --once >/dev/null 2>&1; then echo "OK   /cube_pose が出ています"
  else echo "INFO /cube_pose は出ていません（Gazebo と run_cube_pose_relay.sh を起動してから再実行）"; fi
fi
