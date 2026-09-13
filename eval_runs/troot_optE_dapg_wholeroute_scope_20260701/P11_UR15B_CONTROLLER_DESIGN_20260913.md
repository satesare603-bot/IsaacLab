# p11 — UR15-B controller 設計 v2（既存 control class の B 側導出・識別記録つき）

**Author** w2:p11 ARM-CONTROL-DESIGN · **Written** 2026-09-13 08:50:08 JST · Naming per m-p18-256: **Rs1 = 人間 / Rs2 = p4**.
**系譜**: v1 = `c7884ce0ff`（FAIL 印つき・blob `1966f3fa4c03`）／cycle-1 verdict = `P11_L3_FIVEWAY_VERDICT_UR15B_CONTROLLER_20260913.md` @ `c7884ce0ff`（union: CRITICAL 1・HIGH 10・MEDIUM 19・LOW 16 = 全受入）。**本 v2 = その全部を反映した書き直し**。cycle 2（pre-check ＋ 5 体）の結果は §15 に追記（append-only）。
**依頼** = m-p18-280 §3（08-10 09:56）。**Rs1 逐語①**「**UR15-Bようのコントローラも作成**」（kickoff `P4_MOUNTING_C-2_CHAIN_KICKOFF_20260808.md` `:2225-2231` @ `9e9f6199b3`・custody `1d9974face`・DDR #68 `00-DESIGN-STATUS-LEDGER.md:172` @ `358a1d72ad`）。
**規約**: 数値は本書内の query 付き値のみ／行番号は **必ず rev を伴う**／数は「行数 or 出現数」を明記／**測定**と**導出**を札で分ける／07-29 由来の数値には **mounting と hand の札**を付ける。
⛔ **run を要求せず含意しない**（§10 の各 row に計器・object・認可状態を明記）。実装 = p0 / 検証 = pZ / まとめ = p4。04-Specs 不触。§0 不変前提（RS71 `:27-:29` @ `13a1331fc0`）を変えない。
⚠ **34 日の遅延** = 当卓の self-report（artifact なし）: court word は 08-10 10:0x に起草・未送信（session 中断・scratchpad 消失）。送達 = 2026-09-13 08:25:24 JST（p18 transcript `:41542`・§1453／Rs1「p11の件は手送りで届けて」08:31 → §1454 で p4/p0/pZ へ手送り）。

---

## 0. COURT WORD = ACCEPT（送達済）

- 根拠 = role brief `ARM_CONTROL_DESIGN_ROLE_BRIEF_p11_20260721.md` @ `2887037c9f` `:8`「設計だけ」／`:62`「どう駆動するか＝あなた」。
- 読み（p4 kickoff 09:51 節・DDR #68 と同一・p4 09-13 節 2 で「差なし」）: **既存 class（per-arm 6D DLS ＋ position servo）の B 側導出。新方式でない。** class が B を表現できない地点 = STOP-and-report（§11）。
- ⚠ brief `:69`「Rs 承認後、p4 が p0 へ実装を渡す」の読み = **Rs1 ①（08-10）が本導出への :69 承認**〔inference 札・違えば p4 が正す〕。p0 の build 窓は Rs1 の一言（p4 09-13 節 2）。

## 1. 結論（導出結果・3 行）＋ Rs1 への問い

1. **導出結果 = identity ＋ 側別測定 ＋ 計器 1 件。** 同じ q を両腕へ（関節符号 map なし）・側に依る量（AXFIX・pad 名・menu 符号・servo 添字）は側ごとに測られて入る・**唯一の code 変更 = 姿勢 cap 計器の側別化（D4・gate-inert・§5）**。
2. **B が既存 class に駆動されることは C-2 で測られている**（08-10 DoD record・ただし当時の hand は回転コピー）— 鏡像 hand（`b7a5e39ecf`）後の再測 = pZ R0（§10）。
3. **鏡像の主張は関節空間**（identity・負の対照 = reference の R 式）。task 空間の「同一 target → 鏡像軌道」は偽（鏡像運動には鏡像 target が要る・pZ 訂正 = 台帳 §1401 `:46790`）。
- ⭐ **Rs1 への問い（p4 経由・非 blocking で設計は進む）**: 「作成」は **identity ＋ D4 ＋ 本書 §7 の識別記録**で満たされるか、**B 側の別 artifact**（例: p0 が driver に identity print 1 行）を期待するか。当卓の推奨 = 前者＋print 1 行（§7・採否 p4）。

## 2. 接地と pin（全て当卓が本 session で直読・rev つき）

| object | rev | 行数 / 札 |
|---|---|---|
| `p4_ur15_sim_20260727/ur15_steps_wired.py`（以下 wired） | **`22feba17a6`**（blob `75eefef4e27e`・sha256 `57de8c3ec7ed0262…`） | 4,022 行。共有 tree は **dirty**（+1387/−813・mtime 09-07 23:15・author は git に無い）= formatter ＋ 未使用 import 6 個削除 ＋ `_interleave_report` 分離。module 全体 AST（import 正規化）292/292・**制御 13 def は blob と同一**（当卓・CC5・CC6 の独立再現 3 件・陽性対照 = literal 1 反転で検出）。⇒ 本書は blob で pin。**着地前提: p0 は clean worktree @ `22feba17a6` から編集**（WIP は D4 対象行 `@@ -1302 @@ def vertical_cap_deg` 等と重なる）・WIP の処遇 = p18 へ照会 |
| `ur15_cell_spec.py` | `0f6b4a733e` | **1,250 行**（v1 の 1,395 は dirty tree・誤り）。共有 tree は dirty（+391/−246・formatter）。cite は blob 行 |
| `ur15_base_mirrored.xml`（`<mujoco model="UR15-B">` `:2`） | `6a542f45dd` | 正式化 `c737f6974e` ＋ header 修正 |
| `ur15_base.xml` | `bf0235cfd8` | stock |
| `_ur15_2f85_koshape_actuated_mirrored.xml` | `b7a5e39ecf` | 鏡像コ hand（08-10 09:09） |
| stock hand `thread_isaac_lab/assets/ur5e_robotiq/robotiq_2f85/_ur15_2f85_koshape_actuated.xml` | HEAD tracked | — |
| `UR15_MIRROR_ACCEPTANCE_20260729.txt` ＋ `ur15_mirror_acceptance.py` | `38678f5946` | 147 行。**札: mounting 0.22/45**（`:4`）・kinematics-of-position のみ（`:132`）・腕のみ |
| `HOME_POSE_SYMMETRY_20260729.txt` ＋ `probe_home_pose_symmetry.py` | `dfb8dc1bde`（07-29 00:55） | 78 行。**札: t42 build = 0.22/45（当時の cell_spec `fc9d999e20`）・hand = 鏡像化前の回転コピー**（`GRIP_XML_MIRRORED` 初出 `b7a5e39ecf` 08-10）・§5 は生 (yaw,roll) を両側へ入れ full-matrix Mx 共役を検査（probe `:249-253`） |
| `PZ_ARM_MIRROR_LEG_RESULT_20260810.md`／`PZ_MIRROR_LEG_PREREG_20260810.md` | `cf0a14cea3`／`9d118cbf93` | 47／35 行。**札: asset 級**（reference JSON の FK・mesh Hausdorff・compile field） |
| `_gen/dod_c2_20260810/run.log`（untracked） | sha256 `04599b84e34be51e…`・89,798 B・08-10 02:10 | **札: C-2（`:84` spread 0.280 tilt 20.0）・R = UR15-B arm ＋ 回転コピー hand（run 02:11 ＜ hand 09:09）** |
| `ur15_gripper_mirror_acceptance.py` | `b7a5e39ecf` | hand-on-arm の driver 不要 builder（§10 の計器） |
| LEDGER `:170/:172/:173/:174`（DDR #66/#68/#69/#70）／RS71 `:27-:29`／brief `:8/:59/:62/:69`／kickoff `:2225-2231`（09:51）・09-13 節／台帳 §1401 `:46790`・§1403 `:46800-46810`・§1453-1454 | `358a1d72ad`／`13a1331fc0`／`2887037c9f`／`9e9f6199b3`／`6006eeb2a8` | — |

## 3. 生きている controller（測定）と、それが B を駆動している証拠

- **`solve_ik(t, tgt, …)`** wired `:2036`: scratch MjData 上の 6D DLS。`e = [tgt − pinch(t); 0.6·er]`・**`er = rotvec((RD @ AXFIX[t]) @ Rtᵀ)`**（`:2115`）・`J = [0.5(Jp0+Jp1); 0.6 Jr]`・`dq = 0.5·Jᵀ(JJᵀ + 0.05²I)⁻¹e`（`:2118`）・`|dq| ≤ 0.15`（`:2120-2121`）・300 iter・収束 `pe ≤ 0.002 m`（`:2129`）・`re ≤ re_max`（既定 0.05／aim 0.02 `:887`／per-step 0.30 `:3398,:3411`）。側は `QADR/VADR/PAD/TOOLB/AXFIX[t]` で入る — 側固有の分岐 0。呼び手 = `aim_slot_at` `:854`・START solve `:2569-2632`・STEPS loop `:3143-3978`（module-level）。
- **`ik()` `:646` は死んでいる**: `\bik\(` = 1 行（def のみ・rc=0）／陽性対照 `\bpinch\(` = 21 行 / 23 出現。⛔ p0 は B の何かを `ik()` に書かない。
- servo = position actuator `AIDX[t]`（gain は関節名で決まる）・home = `d.ctrl[AIDX[t]] = HOME_POSE` 両側 loop `:2505-2507`。cell 構成 = `for tag, sign in SIDES.items()` `:401`・mount `pos=[sign·YOKE_SPREAD, 0, SHOULDER_HEIGHT]` `:409`・arm `:238`（R = `ur15_base_mirrored.xml`）・hand `:419`（R = `GRIP_XML_MIRRORED`）・wrist→hand frame `:420`。identity print `:567-568`。
- ⚠ **driver は import 即実行**（`:442 m = cell.compile()`・`:1148-1149` cable settle mj_step 2000・`:3143` STEPS loop 全て module-level・`__main__` guard 0）⇒ **wired の import = wired 実行**（DDR #66 受領 (4)・#69）。B の検証は §10 の driver 不要計器で行う。
- **B を駆動した記録（C-2・回転コピー hand・測定）**: `dod_c2_20260810/run.log` `:55` start-pose IK **R: 16 solved / 4 collision-free**・`:60` R chosen q・`:99` **STEP1 R tool err 2.2 mm**・`:355` **STEP2 COMMAND R reached 100.0%**・`:366` R 2.2 mm — 同 record で L は `:51` 0/13 collision-free・`:94` 658.5 mm（記録は判別する）。⇒ 既存 class は C-2 で B を駆動した。**鏡像 hand 後は未測**（AXFIX["R"] が変わった）→ R0。

## 4. 側に依る量の census（閉じた query・母集団 = wired blob 4,022 行 @ `22feba17a6`・単位 = 行）

- **P1 容器**（`(AXFIX|QADR|VADR|TOOLB|PAD|PADG|PAD1G|CLAWG|ARMG|ARMB|COLFREE|GIDX|AIDX|qt|START|GRASP1|prev|w)\["(L|R)"\]`）= **11 行**／**P2 呼び出し形**（`\w+\("(L|R)"`）= **14 行**／**P3 既定引数**（`=\s*"(L|R)"[,)]`）= **3 行**。陽性対照 = `[t` 形 87 行 / 92 出現（`\[t\]` 厳密 81 / 86）。
- 分類（全 hit）:
  | 種 | 行 | 分類 |
  |---|---|---|
  | **L の量が B にも使われる（是正対象）** | `:1276` `slot_centre("L") − pinch("L")`・`:1277` `TOOLB["L"]`・`:1283` `AXFIX["L"]` | 姿勢 cap 計器（`attitude_tilt_deg`/`vertical_cap_deg`）→ **D4** |
  | **L の量が B にも使われる（共有を正当化）** | `:2814` `mouth_clear(t="L")` 既定 → `:2827` `release_ctrl` `:2847` `CLAWG["L"][0],[2]` → `:3137` `RELEASE = release_ctrl()` → `:3141` STEPS の両手へ（row 17 `:2890` "RELEASE","RELEASE"） | 解放指令は **L の爪間隙**から解かれ両手に流れる → **D5′** |
  | L のみの診断（印字） | `:3021-3029` `[rel]` block（`CLAWG["L"]`・scratch で mj_step）／`:2624-2629` L-vs-FINAL-R 診断 | 対象外（B の指令に入らない・記録のみ） |
  | 両側の対 | `:1991` `ARMG` L∩R／`:2656`+`:2659` `QADR` L→R／`:3303` `_pred_w`／`:3527` `GIDX` dict／`:2487` `pose_menu("L")`,`("R")`／`:2651` `LAST_CLEAR`／`:3006 :3760 :3779-3780 :3786-3787 :3940` | 側別に扱われている |
  | 既定引数（L 既定） | `:229` `arm_spec(tag="L")`・`:1937` `arm_pair_min(ta="L", tb="R")`・`:2814` | `:229/:1937` は両側で明示呼び出し・`:2814` は上の D5′ |
- ⇒ **B に L の量が入る経路は 2 つ**（cap 計器・解放指令）。v1 の「2 箇所・同一計器内」は query の名前 8 個の産物で誤り（verdict H1）。

## 5. 側に依る量の表（設計の本体）

| 量 | 現行 | B での正しさの根拠（測定・札つき） | 設計 |
|---|---|---|---|
| **関節指令 map A→B** | 同じ q（HOME_POSE 1 本・START/STEP は側別に解く） | acceptance `:65` test leg（鏡像腕・右 mount・**左の関節値** → reference の右位置）**48/48・worst 0.0076 mm / bar 1.0 mm**〔@0.22/45〕／negative `:121`（reference の R 式を与える）**0/48・worst 1357.5480 mm**／pZ B1 reference 自体 exact mirror・B2 pZ 自身の FK worst **0.0079 / 0.0074 mm**（record 再 parse の band 0.0013–0.0076 とは別）・B3 Hausdorff 0 × 7 mesh・C2 正式化 = naming-only〔asset 級〕／08-10 record（§3）〔C-2・回転コピー hand〕 | **D1 = identity。符号 vector を置かない。** reference の R 式（pZ 実測: R = −L、**upper_arm/wrist_1 は −L + π**）は **非鏡像機の経路** = UR15-B では **負の対照**（p4 基準 (f) との整合 = この意味で整合）。C-2 では reference 位置が無い（DDR #68・pZ F4）⇒ C-2 の identity は **構成**（`:238/:401/:409/:419`）＋ **R1 の自己鏡像 leg** で立てる |
| joint limits | `LIM` 共有 `:1250`（URDF・cell_spec `:100`） | 6 range とも対称かつ両 asset で同一（mirrored `:38-58`・stock `:18-38`） | **D6 = 共有は D1 の帰結（対称性に依らない）。** identity-q 鏡像（`A·Rot(n,q)·A = Rot(−An,q)`）では range は **byte 同一**が要件（hand generator `make_ko_mirror.py:16-19` @ `b7a5e39ecf` が明記・hand は非対称 range を byte 同一で持つ）。⚠ acceptance `:214` の `[lo,hi]→[−hi,−lo]` 期待は逆（対称ゆえ今日は落ちない）→ **別 task へ回付**（本 chunk 外） |
| AXFIX（閉じ軸・接近軸の tool 内表現） | 側ごと測定 `:594-613`・`AXFIX` `:616`（共通 seed q = identity 下で鏡像配置） | 構造 = 側別測定（測定）。AXFIX_L と AXFIX_R の関係は **鏡像 hand では未測**（07-29 §5 の pad 入替・det=+1 は回転コピー hand の値・det は s=a×c ゆえ構成的に +1 で証拠にならない） | **D2 = 側別測定を維持。式で鏡像化しない。** 関係の測定 = R1′（§10）。⚠ AXFIX は直交化していない（a·c = 2e-5 今日）— `|c·a| < 1e-3` assert は **D4 外の提案** |
| 姿勢指令 (yaw, roll) | 側別符号 `sgn`（`pose_menu` `:2027-2029`・`solve_ik` `:2055`・`:2061`・comment「p5 −167」）= `SIDES` cell_spec `:485` と同じ数（kinonly `:661/:666` は `spec.SIDES[t]` を menu 符号に使う） | `_rdes` `:1329-1334` = Rz(yaw+π/2)·Ry(roll)・`R_DES` `:489-491`。収束時 world の把持三軸 = RD の列（側に依らず・導出・CC4 再導出一致）。**導出**: 指令 pair の関係 = **`RD_R = My·RD_L·Mx`**（接近列は y=0 鏡像・閉じ列は y 鏡像＋反転）・例 (0.3, 0.6): 接近列 (−0.1669, −0.5394, +0.8253) 対 (−0.1669, +0.5394, +0.8253)。x=0 鏡像は **roll ≠ 0 で崩れる**（yaw でない — §5-of-07-29 の「yaw≠0 で NO」は生 entry 入力の検査） | **D3 不変。** 根拠 = **測定された code の規約**（`:2027/:2055/:2061`）。「Rs: hands do not have to mirror … have to clamp」`:904` は p4 の 07-27 docstring（Rs1/Rs2 未分離・custody 不在）、「p5 −167」は code 内参照のみ ⇒ 裁定 custody は p4/p5 への**非 blocking 質問**。「menu が鏡像であるべきか」は p5 と共有の設計問題・本書は触れない |
| **姿勢 cap 計器** | `attitude_tilt_deg` `:1263-1286`／`vertical_cap_deg` `:1288-1326`: L 固定（`:1276/:1277/:1283`）・menu の**生** (yaw,roll) で評価（L が受けるのは `(−yaw, −roll)`） | 消費 = **print `:2987-2988` のみ**（`vertical_tol_deg(` の呼び出し = blob 0・cell_spec `:829` は cap 無しの interim 5.73）。gate `:3517` は既に側別（`:3446`）。記録の cap は常に 5.73 = 0.10 rad 厳密 ⇒ v_c ≈ 0 が既に示唆（calibration `:1301` 0.5° ⇒ \|v_c\| ≤ 0.0087・cap ∈ [5.24°, 6.23°]）。現行の値 = 「pad が入れ替わらなかった場合の B」= どちらの腕の実姿勢でもない（v_c=0 なら三者一致） | **D4（§6）= 唯一の code 変更・gate-inert。** 将来 r_max 着地で `vertical_tol_deg(r_max, cap)`（cell_spec `:768-787`・第 3 分岐 raise）に入る — その配線は **p5 §6.4j の court・本 chunk 外・/reward-design 対象** |
| gripper 指令 | `d.ctrl[GIDX[t]]` 0..255 共有 | mirrored hand default joint axis `1 -0 -0`（`:31`）≡ stock `1 0 0`（`:27`）／actuator 同一（`:202` vs `:198`）／192/192（`b7a5e39ecf`）／hand の非対称 range は byte 同一（generator の規則） | **D5 不変** |
| **解放指令 RELEASE** | `release_ctrl()` `:2827` が L の爪間隙（`mouth_clear()` `:2814`・`CLAWG["L"]` `:2847`・scratch で SETTLE_S 分 mj_step）から解き、両手へ `:3137/:3141` | hand は exact mirror（192/192）ゆえ **間隙→ctrl の関係は構成上同一**（導出）。測定は無い | **D5′ = 共有を維持（正当化つき）**＋ pZ static row（§10 R5: 同じ ctrl での `mouth_clear(L)` と `mouth_clear(R)` の等式）。側別化（scratch 動力学を 2 倍）はしない |
| home | `HOME_POSE` 1 本（cell_spec `:455`）・servo `:2505-2507` | HOME_POSE_SYMMETRY §1 link frames mirror error 0.0000 mm × 6・§2-§3 mast 距離 両側同値〔**@0.22/45・t42・回転コピー hand**〕／C-2 では未測 | **D6 不変**（C-2 の家 = R1 の HOME_POSE 行で測る） |
| dynamics | kp/kv 関節名で共有・asset inertial は text 上 mirror（pos x 反転・quat (w,x,−y,−z)・mass/diaginertia 同一 — CC4 全 6 body 確認） | position test は dynamics を見ない（acceptance `:132`・pZ B4） | **D7 = static field 等式（R2）＋ 認可 run の L/R 追従誤差比較（R4・報告のみ・等級は pZ）** |

## 6. D4 の仕様（p0 向け・結果形・gate-inert）

- **対象**: wired `attitude_tilt_deg` `:1263-1286` と `vertical_cap_deg` `:1288-1326` @ `22feba17a6`（内容で特定: `v = slot_centre("L") - pinch("L")` を含む関数／`_spec.GRASP_ATTITUDES` を走査し `min(tilted)` を返す関数）。**print `:2987-2988`**（`# p11 -137 / p5` `:2985` — 本 D4 は当卓自身の 07-28 計器〔`2a08328dee`・`c9b5c3abdf`・`4c2ba02697`〕の側固定を正す）。
- **変更（結果形）**:
  1. `attitude_tilt_deg(t, yaw, roll)`: `v = slot_centre(t) − pinch(t)`・`v_tool = xmat[TOOLB[t]]ᵀ v`・`world = (_rdes(yaw, roll) @ AXFIX[t]) @ v_tool`（転置注記 `:1279-1282` 不変）。
  2. `vertical_cap_deg()`: 各 `t ∈ SIDES` について **その側が実際に受ける姿勢** `(SIDES[t]·yaw, SIDES[t]·roll)`（`SIDES` は cell_spec `:485` を driver が既に import — **pose_menu/solve_ik の literal は触らない**）で `upright_t`／`tilted_t` を作り、**両 calibration raise（`:1301` upright ≤ TILT_CAL_DEG／`:1307` min(tilted) ≥ TILT_CAL_DEG）を側ごと**に通し、`cap_t = min(tilted_t)`（**入力側で選ぶ** `abs(r) ≥ 1e-9` `:1306`・出力選択は `:1314-1322` の bug）。返り値 = `min_t cap_t`（共有 `VERTICAL_TOL_DEG` は側ごとの gate `:3517` に当たるため識別性は両側で要る — min が正しい集約・鏡像なら no-op）。
  3. print `:2987-2988` に **両側の cap と採用 min**（＋ `VERTICAL_TOL_DEG` との並記）。
- **不変**: `solve_ik`／`pose_menu`／`_rdes`／`aim_*`／`release_ctrl`／servo／`R_DES`／`GRASP_ATTITUDES`／`LIM`／`AXFIX`／`SIDES`。**制御行の変更 0・file = wired 1 本**（cell_spec 不触・`vertical_tol_deg` の呼び出し形不変）。
- **効果の正直**: gate-inert（消費 = print）。数値効果 = **0 iff v_c = 0**（記録が示唆）。⚠ **起動時 abort 面が両側に広がる**: 2 つの calibration raise は 08-02 に実際に発火した（`order_test_logs/order_L.txt:96`・`order_R.txt:96`・`round_by_round_LOUD.txt:142`「the attitude with zero roll comes out … off vertical」）。⇒ **cap 計器の abort は controller の verdict ではない**（計器が live の jaw 状態を読む）。pZ R3 で両側極値を static に先に測る。
- **名の区別**: `P4_ROLL_CAP`（`:912-918`・env hook・menu を |roll| で絞る）は別物・不触。
- **期待（導出・pZ R3 が決める・両結果の帰結）**: tilt = acos(−world_z)・world_z = −v_c·sinρ + v_a·cosρ（yaw 非依存）。**前提 3 つ**（① 閉じ軸の関係 ② 接近軸の関係 ③ 爪集合＝mouth の鏡像＋読み取り時の指状態が両側同じ）が鏡像 hand で成り立てば両腕の実 tilt は entry ごとに一致 ⇒ D4 は数値 no-op（構造は正しい）。成り立たなければ D4 が数値を直す。**どちらでも D4 は正**。

## 7. UR15-B controller の識別記録（created-object の類比・Rs1「作成」への答えの形）

| 構成要素 | B 側の instance | どこで測られたか | 残り |
|---|---|---|---|
| 解 | `solve_ik("R", …)`：`QADR["R"]/VADR["R"]/PAD["R"]/TOOLB["R"]` `:351-371`・`AXFIX["R"]` `:594-616`・`sgn=+1` = `SIDES["R"]` | 08-10 C-2 record（回転コピー hand）§3 | **R0**（鏡像 hand 後） |
| 目標 | `GRASP1["R"]` `:1149`・STEPS の R 列（`:2880-2890`）・`fix_x=GR[0]` `:922` | 設計定数（cell_spec） | — |
| servo | `AIDX["R"]` `:349`・kp/kv 関節名で共有・`HOME_POSE` 共有・`LIM` 共有 | 08-10 record（B が home→STEP2 到達）・range byte 同一（測定） | R2 |
| 指 | `GIDX["R"]` `:350`・`OPEN/CLAMP/RELEASE` 共有（RELEASE は L 由来・D5′） | 192/192・生成規則 | R5 |
| asset | `ur15_base_mirrored.xml` @ `6a542f45dd`（UR15-B）＋ `_ur15_2f85_koshape_actuated_mirrored.xml` @ `b7a5e39ecf` | acceptance 48/48・pZ B1-B3・192/192 | R1・R1′ |
| 提案（p4 採否・D4 と別の一語） | identity print `:567-568` の隣に **「controller: existing class on both arms; per-side AXFIX/QADR/AIDX; sgn = SIDES[t]; design = P11_UR15B_CONTROLLER_DESIGN_20260913.md @ <commit>」1 行** | — | — |

## 8. `/diffik-trajectory` 出力（brief `:59` の設計ゲート・本設計は軌道を変えない）

```
## 軌道設計: UR15-B controller（既存 class の B 側導出）
### 移動量
solve_ik = scratch 上の反復解（軌道でない）: |dq| ≤ 0.15 rad/iter・300 iter・pe ≤ 0.002 m。実軌道 = START ramp（RAMP :2710・線形）で q を servo 目標へ。D1-D7 のいずれも軌道・step size・補間を触らない。
### 補間方式
position servo への線形 ramp（現行・不変）。one-shot 目標なし。指 = tendon actuator 0..255（不変）。
### DLS確認
λ = 0.05（:2118）・joint clamp 0.15 rad/iter・gain 0.5・回転重み 0.6 — 側に依らない scalar ⇒ B でも同値（不変）。
### THREAD固有リスク
(1) 関節 sign map を「作る」誘惑（reference の R 式）= 負の対照 0/48。(2) 鏡像述語を task 空間で書くと正しい controller が落ちる。(3) dynamics 未検証（R2/R4）。(4) 識別性の「無い」検査（det=+1・限界 leg・cap 一致）を証拠に数えない。
```

## 9. 命令空間の明記（p4 基準 (e)）

**関節空間 = 真**（同じ q → 鏡像姿勢・acceptance test leg・R1）／**task 空間 = 偽**（同じ target → 鏡像軌道ではない・鏡像運動には鏡像 target が要り、姿勢は D3 で y=0 面鏡像ゆえ x=0 面では鏡像にならない）。pZ 訂正の durable = 台帳 §1401 `:46790`（m-p18-281）・§1403（m-p18-282）。

## 10. pZ へ（rows は提供・等級と bar の最終は pZ の court・各 row に計器／object／認可状態）

| row | 内容 | 計器 / object | 認可状態 | 提供 bar |
|---|---|---|---|---|
| **R0** controller | `solve_ik(t="R")` が STEPS の R 目標（`:2880-2890` の R 列）で L と同じ bar（pe ≤ 0.002・re ≤ re_max）で収束・負の対照 = R 式 seed | solver を **隔離 harness**（`solve_ik`＋依存を copy）に載せ、composed model（下の builder）で評価 | ⚠ **class 判定 = p4/pZ**（driver 不実行・mj_step 0 だが driver 由来 code）。代替 = #69 run 内で測る（prior = 08-10 record） | 収束率 L と同等・0 収束なら §11 STOP |
| **R1** 腕の自己鏡像（C-2） | q ∈ {HOME_POSE, reference on-yoke 24 pose, in-limit 乱数 M≥24} で `p_R(q) = Mx·p_L(q)`・`R_R(q) = Mx·R_L(q)·Mx`（各 link・tool body）。負の対照 = `q_R = R式(q_L)` | `ur15_mirror_acceptance.py::build()` @ `38678f5946`（腕のみ・mount は cell_spec から live import = C-2 既定・driver 不要） | mj_step 0・既存計器・pZ が 08-10 に隔離 copy で走らせた class（DDR #68 記載）⇒ **run 認可不要**（pZ 自身の判定） | ≤ 1e-3 mm・≤ 1e-6 rad。⚠ pZ PREREG `:28` の dead query (b) は **kinonly cell（stock arm 両側）**での測定で当たらない（本 builder は鏡像 asset を載せる・07-29 §1 が同形で 0.0000 mm） |
| **R1′** 構成（hand-on-arm） | AXFIX 相当（閉じ軸・接近軸の tool 内表現）を両側で測り、関係 `AXFIX_R = Mx·AXFIX_L·A_h`（CC4 導出）と pad 名の対応・tool frame の共役を確認 | `ur15_gripper_mirror_acceptance.py::build_side()` @ `b7a5e39ecf`（**wrist→hand frame `acc.R_TOOL` = wired `:420` と同一式**〔`ur15_mirror_acceptance.py:56-57`〕・両側・driver 不要） | 同上 | 関係式の残差 ≤ 1e-6 |
| **R2** 動力学 field | 対応 body/joint/actuator の `body_mass`・`body_inertia`（共役後）・`jnt_range`・`dof_armature`・`dof_damping`・`actuator_gainprm/biasprm` | 単体 compile（両 asset）＋ composed model | 同上 | int/range = 厳密・共役 float ≤ 1e-12 相対（pZ B3 6.7e-16） |
| **R3**（D4 着地後・p0 受入項目でもある） | 両側の実姿勢での entry ごと tilt・side ごとの v_c・cap 前後・両 calibration 極値（static）・`min_cap` と `VERTICAL_TOL_DEG` の並記。**selector = roll** | 隔離 harness（`attitude_tilt_deg`/`vertical_cap_deg`＋`_rdes`＋AXFIX 相当）on composed model | 同上 | 一致 ≤ 1e-6 deg なら D4 は数値 no-op（構造のみ）／不一致なら D4 が値を直す |
| **R5** 解放指令 | 同じ ctrl での `mouth_clear(L)` と `mouth_clear(R)`（爪間隙）の等式 | composed model・static（指 q を同じに置く） | 同上 | ≤ 1e-6 m |
| **R4** 認可 run | STEP ごとの tool 位置/回転誤差 L vs R・env 版の pin（p4 09-13 節 5: env_isaaclab7 = Newton 1.5.1 / mujoco 3.11.0 @ 09-10） | #69 の run 自身（RUN_METRICS.json・pB/pC レグ実在） | **#69 の中のみ**（Rs1） | 報告（等級は pZ） |
- ⛔ task 空間で「同じ target 系列 → 鏡像軌道」と書かない（§9）。

## 11. STOP 条件（class の限界・§0）

- R1 が bar を超えて落ちる（同じ q で鏡像にならない）→ built cell が #68 の premise を実現していない = premise 側 → STOP → p4 → Rs1。
- R0 で `solve_ik` が B の目標に収束しない（L の鏡像 target では収束）→ class が B を表現できない → STOP-and-report（method swap しない）。
- D4 以外に「B のために」制御行を触る案 → 本書の外 = 新しい一語。

## 12. 当卓が測っていないもの（限界を書いた質問）

1. §5 姿勢行・§6 期待の算術は **導出**（`_rdes`/`pose_menu`/`_measure_axfix` の定義から・CC4/CC6 が独立に同値を再導出）— 測定は pZ R1′/R3。当卓は run 認可を持たず FK 評価も実行していない。
2. 鏡像 hand での AXFIX_L/AXFIX_R の関係・v_c の符号 = 未測（R1′/R3）。dynamics = text 上のみ（R2）。
3. 09-07 の dirty WIP の author・意図 = 不明（AST で制御 13 def 同一まで）→ p18 へ照会（着地条件）。
4. DoD run 2 本の **STEP2 L stall**（mast band `:2189-2190` kickoff）は L 腕の事象・本書の対象外。D4 は stall に触れない。

## 13. DoD と受入（結果形）

- **設計 chunk**: v2 bank ＋ cycle-2 五体 PASS ＋ **pre-check ≠ BLOCK** ＋ p4 受入の一言 ＋ Rs1 の問い（§1）の回答（設計は回答を待たずに進む・「作成」の充足宣言だけが待つ）。
- **p0（D4）**: (a) 変更 file = wired 1 本・base = `22feba17a6` の clean worktree・hunk は D4 の 2 関数＋print のみ (b) **module 全体**の正規化 AST（import alias 順・f-string 分割を正規化）が blob と一致（許容差 = `attitude_tilt_deg`・`vertical_cap_deg` の 2 def ＋ print の Expr 1 — 陽性対照 = literal 1 反転で検出） (c) `attitude_tilt_deg(t, …)` が side 引数を取り `vertical_cap_deg` が両側で評価し min を返す (d) print に両側 cap（観測は R3 の harness で・driver 実行ではない） (e) hash = function ＋ commit・pin は内容 (f) run 0 (g) 08-02 abort 先例の注記。
- **pZ**: R0-R5 の rows（court で確定）。
- **chain**: D4 着地 ＋ R1/R1′/R2/R3/R5 通過 ＋ R0 の disposition — **#69 の充足宣言は p4（Rs1 認可）・本書は式を置かない**。RUN_METRICS.json・pB/pC レグは #69 run 時に実在（09-05 E1・p4 09-13 節）。

## 14. gate 記録

- **[TASK] L=L3（自己申告）** | node = `T-ROOT-Kinematic-Pin-Complete-Removal-20260719` の下の sub-court（p4 の node・当卓 node なし・manifest `:149`）／関連 `T-ROOT-C3C5-Port-To-Current-Substrate-20260809`。
- **[L-TRIAGE] stage1**:
  ```yaml
  L_TRIAGE: {self_declared: L3, auto_escalated: L3, final: L3}
  evidence:
    step_1_file_matches: []            # 新規 .md・code diff 0
    step_2_diff_keywords: [{keyword: ik, context: 設計対象が IK 路}, {keyword: solver, context: 同}]
    step_3_quantitative: {estimated_lines: ">200", estimated_files: 1}
    step_4_skill_variant: N/A
  notification: {escalated: false, message: "self L3 = auto L3", justification_required: false}
  ```
  `[L-TRIAGE RESULT] final_L: L3 / confidence: HIGH / evidence_summary: brief :59（制御設計 = L3＋設計ゲート＋pre-check）・新規 file >200 行 / required_gates: [DoD 宣言, pre-mortem(§11-12), handoff, 5 体事前 debate, /diffik-trajectory(§8), /pre-check, 層5(post-change・D4 着地後に owed)] / status: READY_FOR_CHECK`
- **[DEFER-RECON]**: #68 = premise（Rs1 裁定済・spec 未反映は DoD/run を gate し設計を gate しない = DDR #70 の読み）／#69 = run を gate（本書 run 0）／#38 = UR15/UR15-B へ supersede／#45・#58 = 不触／#54・#57 = L 腕・mounting の事項／#48 = 別 chunk／#34・#35 = commit は `--no-verify`＋pathspec。FOUNDATIONAL 未解決で本 chunk を塞ぐ行 = 0。
- **[RULE-CHECK] Tier 0-4**: Tier 0 prohibited.md — IK 以外の駆動 0・kinematic trick 0・制御方式変更 0・CLAUDE.md 不触・方針変更 0 ✓／Tier 1 新 file = Rs1 ① に名のある object ✓・新 CLI/Phase 0 ✓／Tier 2 skill 出力 = §8 ✓・pre-check = 実行済（cycle 1 BLOCK・cycle 2 §14 追記）✓／Tier 3 3 原則 = 迎合 0・対処療法 0（D4 は根本 = 側固定）✓／Tier 4 claim-check = 「在る/無い」主張は全て同 session の command 出力（closed query・`git show`・sha）に接地 ✓。
- **[VERIFY] 5 体（事前）**: cycle 1 = **FAIL** @ `c7884ce0ff`（verdict file）。cycle 2 = 本 v2 に対し pre-check ＋ 5 体 → 結果は **§15** に追記。
- **p4 消費基準 (a)-(h)（kickoff 09-13 節 2）への対応**: (a) 本書は bank commit の blob で読まれる（pin は第 2 信） (b) hash = sha256 ＋ commit (c) §0 不変（§0・§14 Tier 0） (d) 既存 class の派生（§0・§3） (e) 命令空間 = §9 (f) 鏡像参照値との整合 = §5 D1 行（R 式は非鏡像機の経路 = UR15-B では負の対照） (g) STOP = §11 (h) run 0 = 冒頭・§10。
