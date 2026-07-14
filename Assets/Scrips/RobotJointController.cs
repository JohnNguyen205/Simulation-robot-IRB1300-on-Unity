using UnityEngine;
using UnityEngine.UI;
using TMPro;
using System.Collections;

public class RobotJointController : MonoBehaviour
{
    [Header("Kéo 6 ArticulationBody link_1 -> link_6")]
    public ArticulationBody[] joints = new ArticulationBody[6];

    RobotKinematics kin;
    RobotUIBuilder ui;
    float[] currentAngles = new float[6];
    GameObject eeMarker, targetMarker;
    bool isMoving = false;

    void Start()
    {
        var oldCtrl = GetComponent("Controller");
        if (oldCtrl != null) Destroy(oldCtrl);

        // Force Stiffness/Damping cho tất cả 6 khớp
        for (int i = 0; i < 6; i++)
        {
            var drive = joints[i].xDrive;
            drive.stiffness = RobotConfig.DRIVE_STIFFNESS;
            drive.damping = RobotConfig.DRIVE_DAMPING;
            drive.forceLimit = RobotConfig.DRIVE_FORCE_LIMIT;
            drive.target = 0f;
            joints[i].xDrive = drive;
        }

        Transform canvas = GameObject.Find("Canvas").transform;
        kin = new RobotKinematics(joints, joints[5].transform, transform.position);

        ui = new RobotUIBuilder();
        ui.BuildAll(canvas);

        for (int i = 0; i < 6; i++)
        {
            int idx = i;
            ui.jointSliders[i].onValueChanged.AddListener((v) => SetJointAngle(idx, v));
        }

        ui.goToButton.onClick.AddListener(OnGoToClicked);
        ui.resetButton.onClick.AddListener(OnResetClicked);
        ui.homeButton.onClick.AddListener(OnHomeClicked);

        CreateMarkers();
    }

    void Update()
    {
        // F9: chup man hinh luu vao thu muc docs/ (dung lam anh minh hoa README)
        if (Input.GetKeyDown(KeyCode.F9))
        {
            string dir = System.IO.Path.Combine(Application.dataPath, "../docs");
            System.IO.Directory.CreateDirectory(dir);
            string file = System.IO.Path.Combine(dir, $"sim_{Time.frameCount}.png");
            ScreenCapture.CaptureScreenshot(file, 2); // 2x supersample cho net
            Debug.Log("[Screenshot] Da luu: " + System.IO.Path.GetFullPath(file));
        }

        if (joints[5] != null)
        {
            eeMarker.transform.position = joints[5].transform.position;
            var pos = kin.GetEndEffectorPosition();
            var rot = kin.GetEndEffectorRotation();
            ui.pxText.text = $"{pos.x:F2} mm";
            ui.pyText.text = $"{pos.y:F2} mm";
            ui.pzText.text = $"{pos.z:F2} mm";
            ui.rxText.text = $"{rot.x:F1}°";
            ui.ryText.text = $"{rot.y:F1}°";
            ui.rzText.text = $"{rot.z:F1}°";
        }
    }

    void SetJointAngle(int index, float angleDeg)
    {
        var drive = joints[index].xDrive;
        drive.target = angleDeg;
        joints[index].xDrive = drive;
        currentAngles[index] = angleDeg;
        ui.jointValueTexts[index].text = $"{angleDeg:F1}°";
    }

    void OnGoToClicked()
    {
        if (isMoving) return;
        if (!float.TryParse(ui.ikXInput.text, out float x) ||
            !float.TryParse(ui.ikYInput.text, out float y) ||
            !float.TryParse(ui.ikZInput.text, out float z) ||
            !float.TryParse(ui.ikRollInput.text, out float roll) ||
            !float.TryParse(ui.ikPitchInput.text, out float pitch) ||
            !float.TryParse(ui.ikYawInput.text, out float yaw))
        {
            ui.ikStatusText.text = "<color=#ff6b6b>Nhap so hop le!</color>";
            return;
        }

        Vector3 tPos = new Vector3(x, y, z);
        Vector3 tRot = new Vector3(roll, pitch, yaw);

        if (!kin.IsInWorkspace(tPos, out string reason))
        {
            ui.ikStatusText.text = $"<color=#ff9800>Ngoai tam voi: {reason}</color>";
            targetMarker.SetActive(false);
            return;
        }

        targetMarker.SetActive(true);
        targetMarker.transform.position = kin.BaseToWorld(tPos);
        ui.ikStatusText.text = "<color=#87ceeb>Dang tinh IK...</color>";
        StartCoroutine(SolveAndMove(tPos, tRot));
    }

    IEnumerator SolveAndMove(Vector3 tPos, Vector3 tRot)
    {
        isMoving = true;
        yield return null;

        // Chỉ sao chép mảng, giữ nguyên góc Unity để truyền vào làm điểm tựa chọn nghiệm gần nhất
        float[] anglesForIK = (float[])currentAngles.Clone();

        var result = kin.SolveIK(tPos, tRot, anglesForIK);

        if (!result.converged)
        {
            ui.ikStatusText.text = "<color=#ff6b6b>Khong tim thay nghiem IK hop le!</color>";
            for (int i = 0; i < 6; i++) ui.jointIKTexts[i].text = "";
            isMoving = false;
            yield break;
        }

        // Hiển thị và di chuyển
        for (int i = 0; i < 6; i++)
            ui.jointIKTexts[i].text = $"→ {result.angles[i]:F1}°";

        ui.ikStatusText.text = $"<color=#87ceeb>Da tim thay nghiệm tối ưu - Robot đang di chuyển...</color>";

        yield return StartCoroutine(MoveTo(result.angles, RobotConfig.IK_MOVE_DURATION));

        // Chờ vài bước physics để ArticulationBody (PD drive) ổn định hẳn về target
        // trước khi đọc vị trí thực tế - tránh đọc lúc khớp còn đang "đuổi theo" setpoint.
        yield return new WaitForFixedUpdate();
        yield return new WaitForFixedUpdate();
        yield return new WaitForFixedUpdate();

        // XÁC MINH THẬT: so sánh vị trí/hướng end-effector thực tế (đọc từ ArticulationBody
        // đã mô phỏng) với target đã nhập, thay vì tin mù vào result.converged.
        Vector3 actualPos = kin.GetEndEffectorPosition();
        Vector3 actualRot = kin.GetEndEffectorRotation();

        float posErr = Vector3.Distance(actualPos, tPos);
        float rotErr = Mathf.Max(
            Mathf.Abs(Mathf.DeltaAngle(actualRot.x, tRot.x)),
            Mathf.Max(
                Mathf.Abs(Mathf.DeltaAngle(actualRot.y, tRot.y)),
                Mathf.Abs(Mathf.DeltaAngle(actualRot.z, tRot.z))));

        if (posErr <= RobotConfig.IK_TOLERANCE_POS && rotErr <= RobotConfig.IK_TOLERANCE_ROT)
        {
            ui.ikStatusText.text = $"<color=#7fdb9f>Da toi dich! (sai so {posErr:F2} mm, {rotErr:F2}°)</color>";
        }
        else
        {
            ui.ikStatusText.text =
                $"<color=#ff9800>CHUA toi dich! Sai lech vi tri {posErr:F2} mm, huong {rotErr:F2}° " +
                $"(target {tPos.x:F0},{tPos.y:F0},{tPos.z:F0} - thuc te {actualPos.x:F0},{actualPos.y:F0},{actualPos.z:F0}) " +
                $"- kiem tra lai cong thuc IK</color>";
        }
        isMoving = false;
    }

    void OnResetClicked()
    {
        if (isMoving) return;
        for (int i = 0; i < 6; i++) ui.jointIKTexts[i].text = "";
        StartCoroutine(MoveTo(RobotConfig.ZeroPosition, 1.2f));
        targetMarker.SetActive(false);
        ui.ikStatusText.text = "Nhap toa do roi bam GO TO";
    }

    void OnHomeClicked()
    {
        if (isMoving) return;
        for (int i = 0; i < 6; i++) ui.jointIKTexts[i].text = "";
        StartCoroutine(MoveTo(RobotConfig.HomePosition, 1.2f));
        targetMarker.SetActive(false);
    }

    IEnumerator MoveTo(float[] target, float duration)
    {
        isMoving = true;
        float[] start = (float[])currentAngles.Clone();
        float t = 0;
        while (t < duration)
        {
            t += Time.deltaTime;
            float s = Mathf.Clamp01(t / duration);
            s = s * s * (3f - 2f * s);
            for (int i = 0; i < 6; i++)
                ui.jointSliders[i].value = Mathf.Lerp(start[i], target[i], s);
            yield return null;
        }
        for (int i = 0; i < 6; i++) ui.jointSliders[i].value = target[i];
        isMoving = false;
    }

    void CreateMarkers()
    {
        eeMarker = GameObject.CreatePrimitive(PrimitiveType.Sphere);
        eeMarker.name = "EE"; eeMarker.transform.localScale = Vector3.one * 0.02f;
        var r = eeMarker.GetComponent<Renderer>();
        r.material = new Material(Shader.Find("Standard"));
        r.material.color = new Color(1f, 0.2f, 0.2f);
        Destroy(eeMarker.GetComponent<Collider>());

        targetMarker = GameObject.CreatePrimitive(PrimitiveType.Sphere);
        targetMarker.name = "Tgt"; targetMarker.transform.localScale = Vector3.one * 0.025f;
        var r2 = targetMarker.GetComponent<Renderer>();
        r2.material = new Material(Shader.Find("Standard"));
        r2.material.color = new Color(0.2f, 0.8f, 1f);
        Destroy(targetMarker.GetComponent<Collider>());
        targetMarker.SetActive(false);
    }
}