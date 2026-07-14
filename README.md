# Simulation robot IRB1300 on Unity

Đồ án mô phỏng cánh tay robot công nghiệp **ABB IRB 1300** (6 bậc tự do) trên Unity.
Làm được hai việc chính:

- **Động học thuận (FK):** kéo 6 thanh trượt để quay từng khớp, panel hiển thị luôn vị trí + hướng của mũi tool theo thời gian thực.
- **Động học nghịch (IK):** nhập toạ độ và hướng đích, robot tự tính góc 6 khớp rồi chạy tới đó. Chạy xong đọc lại vị trí thật từ physics để báo sai số, không chỉ báo "đã giải xong" cho có.

Mình mô phỏng bằng `ArticulationBody` (khớp vật lý của Unity, có PD drive) chứ không phải xoay Transform tay, nên chuyển động sát servo thật hơn.

---

## Ảnh mô phỏng

Chụp trực tiếp lúc chạy trong Unity:

<p align="center">
  <img src="docs/overview.png" width="80%" alt="Giao diện mô phỏng"/>
</p>

| Bảng điều khiển + robot | Đổi góc khớp, tay cong theo (FK) |
|:---:|:---:|
| <img src="docs/panel.png" width="100%"/> | <img src="docs/pose.png" width="100%"/> |

Toàn bộ bảng điều khiển (slider khớp, ô nhập IK, panel FK, nút GO TO/HOME/RESET) đều dựng bằng code lúc runtime trong `RobotUIBuilder.cs`, không kéo thả trong Editor.

---

## Robot ABB IRB 1300

Dùng biến thể **IRB 1300-11/0.9** (tải 11 kg, tầm với 0.9 m). Vài số chính lấy từ datasheet ABB:

| Thông số | Giá trị |
|---|---|
| Số bậc tự do | 6 |
| Tầm với | 900 mm |
| Tải trọng | 11 kg |
| Độ lặp lại (ISO 9283) | 0.02 mm |
| Khối lượng robot | ~75 kg |
| Bộ điều khiển | ABB OmniCore |

Giới hạn góc và tốc độ từng trục (datasheet):

| Trục | Chức năng | Giới hạn góc | Tốc độ tối đa |
|:---:|---|:---:|:---:|
| 1 | xoay thân | −180° … +180° | 280 °/s |
| 2 | vai | −100° … +130° | 228 °/s |
| 3 | khuỷu | −210° … +65° | 330 °/s |
| 4 | xoay cổ tay | −230° … +230° | 500 °/s |
| 5 | gập cổ tay | −130° … +130° | 420 °/s |
| 6 | mặt bích | −400° … +400° | 720 °/s |

Trong mô phỏng, giới hạn thực tế lấy từ `RobotConfig.cs` (đây mới là nguồn để robot chạy):

```csharp
LowerLimits = { -180, -95, -210, -230, -130, -400 };  // độ
UpperLimits = {  180, 155,   65,  230,  130,  400 };  // độ
```

Có một chỗ lệch datasheet: trục 2 mình để dải **−95°…+155°** (rộng hơn bản 11/0.9) cho robot với tới nhiều tư thế hơn khi test. Nếu cần đúng datasheet thì sửa lại hai dòng trên.

Hai tư thế mốc: **Home** `{0, −30, 30, 0, 60, 0}` và **Zero** (tất cả khớp = 0).

---

## Vùng làm việc

Robot chỉ nhận đích nằm trong vùng với được. Trước khi giải IK, code kiểm tra 3 điều kiện (hệ base, đơn vị mm):

| Điều kiện | Giá trị |
|---|---|
| Bán kính ngang `√(x²+y²)` | ≤ 900 mm |
| Chiều cao Z | 150 … 1450 mm |
| Khoảng cách tới khớp 2 `√(x²+y²+(z−544)²)` | ≤ 900 mm |

Nhập đích ngoài vùng thì panel báo lý do (vượt bán kính / quá thấp / quá cao) và không giải nữa.

<p align="center">
  <img src="docs/workspace-envelope.svg" width="70%" alt="Bao hình vùng làm việc"/>
</p>

---

## Phần toán

### Hệ toạ độ

Unity xài mét, hệ trục trái; robot mình tính bằng mm. Ánh xạ từ toạ độ local trong Unity sang hệ IK:

$$
\mathbf{p}_{IK} = (\,p_z,\; -p_x,\; p_y\,)\times 1000
$$

Hướng mũi tool biểu diễn bằng **Roll–Pitch–Yaw** (ZYX). Từ RPY suy ra vector hướng tiến (approach) của tool:

$$
\hat{\mathbf{a}} =
\begin{bmatrix}
\cos y \sin p \cos r + \sin y \sin r \\
\sin y \sin p \cos r - \cos y \sin r \\
\cos p \cos r
\end{bmatrix}
$$

### Động học thuận (FK)

Lúc đầu mình định dùng bảng tham số DH cổ điển, nhưng đo lại thì các hằng số DH **không** khớp với rig thật trong scene — công thức giải tích sai một cách hệ thống. Nên mình bỏ DH, dựng FK trực tiếp từ dữ liệu neo (`anchor`) của từng `ArticulationBody` đọc thẳng từ file scene. Cách này đã kiểm lại khớp **0.00 mm / 0.0°** với rest-pose lưu trong scene.

Với mỗi khớp $i$, tích luỹ vị trí $\mathbf{p}$ và quay $R$:

$$
\mathbf{p} \leftarrow \mathbf{p} + R\,\mathbf{p}^{\,parent}_i,
\qquad
R \leftarrow R \; R^{\,parent}_i \; R_x(\theta_i)\; \big(R^{\,anchor}_i\big)^{-1}
$$

Xong lấy vị trí mặt bích (flange), đổi sang hệ IK rồi cộng thêm đoạn tool $d_6 = 90$ mm dọc hướng approach để ra mũi tool:

$$
\mathbf{p}_{tool} = \mathbf{p}_{flange} + d_6\,\hat{\mathbf{a}}
$$

> Vì FK nội bộ bám theo `anchor` của scene hiện tại, nếu sau này đổi rig (thêm/bớt link, đổi kích thước) thì phải cập nhật lại mảng `Anchors` trong `RobotKinematics.cs`, không thì IK sẽ lệch.

### Động học nghịch (IK) — giải bằng số

Vì công thức giải tích không khớp rig, mình giải IK bằng số: tối thiểu bình phương sai số tư thế 6 chiều (vị trí + hướng) bằng **Damped Least Squares (Levenberg–Marquardt)**.

Sai số:

$$
\mathbf{e}=
\begin{bmatrix}
\mathbf{p}^{*}-\mathbf{p}(\boldsymbol\theta)\\[2pt]
w\,\boldsymbol\omega
\end{bmatrix}
$$

trong đó $\boldsymbol\omega$ là sai số hướng lấy từ axis–angle của $R^{*}R(\boldsymbol\theta)^{-1}$, còn $w = 300$ mm/rad để cân thang đo giữa vị trí (mm) và hướng (rad).

Jacobian $6\times6$ tính bằng sai phân hữu hạn ($\epsilon = 0.05°$), rồi cập nhật LM với hệ số giảm chấn $\lambda$ tự điều chỉnh:

$$
\big(J J^{\top} + \lambda^{2} I\big)\,\mathbf{y} = \mathbf{e},
\qquad
\Delta\boldsymbol\theta = J^{\top}\mathbf{y}
$$

Bước nào làm giảm sai số thì giảm $\lambda$ (ngả về Gauss–Newton, hội tụ nhanh), bước nào tệ hơn thì tăng $\lambda$ (ngả về gradient descent, an toàn). Lặp tối đa 200 vòng, dừng khi sai số ≤ 1 mm và ≤ 1°.

**Nhiều điểm xuất phát:** IK số dễ kẹt ở cực tiểu địa phương / vùng kỳ dị, nên mình chạy LM từ 9 seed khác nhau (góc hiện tại, Home, Zero, và 6 seed ngẫu nhiên trong giới hạn khớp) rồi chọn nghiệm hội tụ **gần cấu hình hiện tại nhất** cho robot chạy đỡ giật. Nếu không seed nào hội tụ thì lấy nghiệm sai số nhỏ nhất.

<p align="center">
  <img src="docs/ik-pipeline.svg" width="88%" alt="Pipeline IK"/>
</p>

### Xác minh sai số thật

Chỗ này mình khá tâm đắc: sau khi robot chạy tới đích, code **không** tin luôn kết quả solver. Nó chờ vài bước physics cho khớp ổn định, rồi đọc lại vị trí + hướng thật của end-effector từ `ArticulationBody` (đã qua mô phỏng PD drive) và so với đích. Panel hiển thị sai số mm/° thật sự đo được — nếu solver ra nghiệm đúng nhưng physics chưa tới thì vẫn lộ ra ngay.

---

## Cấu trúc code

Toàn bộ trong `Assets/Scrips/`:

| File | Việc |
|---|---|
| `RobotConfig.cs` | Hằng số: giới hạn khớp, tầm với, giới hạn Z, tham số IK và drive, tư thế mốc. |
| `RobotKinematics.cs` | FK (đọc physics + FK nội bộ theo anchor), IK số (LM/DLS), kiểm tra vùng làm việc, đổi hệ toạ độ. |
| `RobotJointController.cs` | `MonoBehaviour` chính: gắn drive cho khớp, nối UI, coroutine giải-di chuyển-xác minh, tạo marker. |
| `RobotUIBuilder.cs` | Dựng toàn bộ UI bằng code lúc runtime (slider, ô nhập, nút, panel hiển thị). |

Luồng chạy IK: nhập đích → `IsInWorkspace` → `SolveIK` (9 seed × LM) → `MoveTo` (nội suy smoothstep) → chờ physics ổn định → đọc lại pose thật → báo sai số.

---

## Cách dùng

| Thành phần | Chức năng |
|---|---|
| 6 slider `Theta 1..6` | Quay từng khớp; panel FK cập nhật X/Y/Z + Roll/Pitch/Yaw ngay. |
| Ô nhập IK | X, Y, Z (mm) và Roll, Pitch, Yaw (độ) của mũi tool. |
| **GO TO** | Kiểm tra vùng làm việc → giải IK → chạy tới → báo sai số. |
| **HOME** | Về tư thế Home. |
| **RESET** | Về tư thế Zero (mọi khớp = 0). |
| Cầu đỏ | Vị trí mũi tool hiện tại. |
| Cầu xanh | Điểm đích IK. |
| Phím **F9** | Chụp màn hình, lưu vào `docs/`. |

---

## Chạy thử

Cần **Unity 2022.3** trở lên (dùng `ArticulationBody` + TextMeshPro).

```bash
git clone https://github.com/JohnNguyen205/Simulation-robot-IRB1300-on-Unity.git
```

1. Mở bằng Unity Hub (Add → chọn thư mục project).
2. Nếu Unity hỏi thì cài **TextMeshPro Essentials**.
3. Mở scene `Assets/Scenes/SampleScene.unity`.
4. Kiểm tra component `RobotJointController` đã gán đủ 6 `ArticulationBody` (link_1 → link_6).
5. Bấm **Play**.

---

## Cấu trúc thư mục

```
.
├── Assets/
│   ├── Scenes/SampleScene.unity   scene mô phỏng
│   ├── Scrips/                     code C#
│   │   ├── RobotConfig.cs
│   │   ├── RobotKinematics.cs
│   │   ├── RobotJointController.cs
│   │   └── RobotUIBuilder.cs
│   └── Materials/
├── Packages/
├── ProjectSettings/
├── docs/                           ảnh minh hoạ
└── README.md
```

`Library/`, `Temp/`, `Logs/`, `.sln`, `.csproj`... do Unity tự sinh, đã bỏ qua trong `.gitignore`.

---

## Còn hạn chế

- IK số phụ thuộc điểm xuất phát; gần vùng kỳ dị đôi khi cần thêm seed mới chắc hội tụ.
- FK nội bộ hiệu chỉnh theo anchor của scene này — đổi rig là phải chỉnh lại mảng `Anchors`.
- Trục 2 để dải rộng hơn datasheet (ghi chú ở trên).
- Chưa mô phỏng va chạm giữa các link với nhau (self-collision).

---

## Tác giả

**JohnNguyen205** — https://github.com/JohnNguyen205

Đồ án học tập. Số liệu robot theo tài liệu *ABB IRB 1300 Product Specification*. ABB và IRB 1300 là thương hiệu của ABB, project chỉ dùng để học, không liên kết với ABB.
