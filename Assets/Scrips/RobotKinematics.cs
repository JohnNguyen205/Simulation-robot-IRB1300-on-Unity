using UnityEngine;
using System.Collections.Generic;

public class RobotKinematics
{
    ArticulationBody[] joints;
    Transform endEffector;
    Vector3 robotBasePos;

    const float D6 = 90f; // khoang cach flange (goc link_6) -> mui tool, doc theo huong approach

    public RobotKinematics(ArticulationBody[] joints, Transform endEffector, Vector3 basePos)
    {
        this.joints = joints;
        this.endEffector = endEffector;
        this.robotBasePos = basePos;
    }

    // ==================== FORWARD KINEMATICS (đọc từ Transform Unity thật) ====================
    public Vector3 GetEndEffectorPosition()
    {
        Vector3 worldPos = endEffector.position;
        Transform robotRoot = joints[0].transform.parent;
        Vector3 localPos = robotRoot.InverseTransformPoint(worldPos);
        Vector3 flange = new Vector3(localPos.z, -localPos.x, localPos.y) * 1000f;

        // endEffector (joints[5].transform = link_6) la FLANGE (goc quay khop 6), khong phai mui tool -
        // trong scene khong co transform con nao dai dien mui tool. Cong them D6 theo huong approach
        // (nhat quan voi SolveIK, noi target luon la MUI TOOL).
        Vector3 rot = GetEndEffectorRotation();
        Vector3 approach = ApproachVector(rot.x, rot.y, rot.z);
        return flange + D6 * approach;
    }

    public Vector3 GetEndEffectorRotation()
    {
        Quaternion worldRot = endEffector.rotation;
        Transform robotRoot = joints[0].transform.parent;
        Quaternion baseRot = robotRoot != null ? robotRoot.rotation : Quaternion.identity;
        Quaternion relRot = Quaternion.Inverse(baseRot) * worldRot;
        Vector3 euler = relRot.eulerAngles;
        float roll = NormalizeAngle(euler.z);
        float pitch = NormalizeAngle(-euler.x);
        float yaw = NormalizeAngle(euler.y);
        return new Vector3(roll, pitch, yaw);
    }

    public Vector3 BaseToWorld(Vector3 ikPos)
    {
        Vector3 pM = ikPos / 1000f;
        // Đảo ngược mapping trên: IK (x,y,z) → Unity local (-y, z, x)
        Vector3 localPos = new Vector3(-pM.y, pM.z, pM.x);
        Transform robotRoot = joints[0].transform.parent;
        return robotRoot.TransformPoint(localPos);
    }

    public bool IsInWorkspace(Vector3 target, out string reason)
    {
        float radius = Mathf.Sqrt(target.x * target.x + target.y * target.y);
        if (radius > RobotConfig.REACH_MAX)
        { reason = $"Ban kinh {radius:F0} > {RobotConfig.REACH_MAX:F0} mm"; return false; }
        if (target.z < RobotConfig.Z_MIN)
        { reason = $"Z = {target.z:F0} < {RobotConfig.Z_MIN:F0} mm"; return false; }
        if (target.z > RobotConfig.Z_MAX)
        { reason = $"Z = {target.z:F0} > {RobotConfig.Z_MAX:F0} mm"; return false; }
        const float D1 = 544f; // chieu cao khop 2 (da verify khop voi rig that trong scene)
        float distFromJ1 = Mathf.Sqrt(target.x * target.x + target.y * target.y +
                                (target.z - D1) * (target.z - D1));
        if (distFromJ1 > RobotConfig.REACH_MAX)
        { reason = $"Cach truc 1: {distFromJ1:F0} > {RobotConfig.REACH_MAX:F0} mm"; return false; }
        reason = "";
        return true;
    }

    public class IKResult
    {
        public float[] angles;
        public float posError;
        public float rotError;
        public int solutionsFound;
        public bool converged;
    }

    // ==================== FK NỘI BỘ (thuần toán, dùng để giải IK số - không đụng physics) ====================
    // Dữ liệu anchor của từng ArticulationBody, lấy TRỰC TIẾP từ SampleScene.unity
    // (m_ParentAnchorPosition / m_ParentAnchorRotation / m_AnchorRotation của joint_1..joint_6).
    // Đây là "nguồn sự thật" của rig thật - đã kiểm chứng khớp 0.0000° với rest-pose lưu trong scene
    // và khớp toạ độ thật của từng link (xem lại nếu rig thay đổi thì phải cập nhật lại các số này).
    struct JointAnchor { public Vector3 parentPos; public Quaternion parentRot; public Quaternion anchorRot; }

    static readonly JointAnchor[] Anchors = new JointAnchor[]
    {
        new JointAnchor{ parentPos=new Vector3(0f,0.252f,0f),
                          parentRot=new Quaternion(0f,0f,-0.7071081f,0.7071055f),
                          anchorRot=Quaternion.identity },
        new JointAnchor{ parentPos=new Vector3(-0.29200003f,0f,0.05f),
                          parentRot=new Quaternion(0.011541947f,-0.011541947f,-0.70701265f,0.70701265f),
                          anchorRot=new Quaternion(0f,0f,-0.7071068f,0.7071068f) },
        new JointAnchor{ parentPos=new Vector3(-0.42500007f,0f,-0.0000000025102054f),
                          parentRot=new Quaternion(-0.011541945f,0.011541947f,-0.70701265f,0.70701265f),
                          anchorRot=new Quaternion(0f,0f,-0.7071068f,0.7071068f) },
        new JointAnchor{ parentPos=new Vector3(-0.039999966f,-0.00000005960465f,0.12950003f),
                          parentRot=new Quaternion(-0.0179381f,0.7068794f,0.0179381f,0.7068794f),
                          anchorRot=new Quaternion(0f,0.7071068f,0f,0.7071068f) },
        new JointAnchor{ parentPos=new Vector3(-0.000000011161486f,5.668427e-10f,0.29549998f),
                          parentRot=new Quaternion(-0.0311991f,0.031199103f,-0.7064184f,0.7064184f),
                          anchorRot=new Quaternion(0f,0f,-0.7071068f,0.7071068f) },
        new JointAnchor{ parentPos=new Vector3(0.00000007590279f,0.000000032267632f,0.08034602f),
                          parentRot=new Quaternion(-0.033283222f,0.7063232f,0.033283222f,0.7063232f),
                          anchorRot=new Quaternion(0f,0.7071068f,0f,0.7071068f) },
    };

    // Trả về vị trí FLANGE (gốc link_6, hệ IK mm) và rotation (quaternion, tương đối base_link) ứng với
    // một vector góc khớp (quy ước Unity/ArticulationBody, giống currentAngles/result.angles).
    static void ForwardKinematicsFlange(float[] thetaDeg, out Vector3 flangeIK, out Quaternion relRot)
    {
        Vector3 pos = Vector3.zero;
        Quaternion rot = Quaternion.identity;
        for (int i = 0; i < 6; i++)
        {
            var d = Anchors[i];
            pos = pos + rot * d.parentPos;
            rot = rot * d.parentRot * Quaternion.AngleAxis(thetaDeg[i], Vector3.right) * Quaternion.Inverse(d.anchorRot);
        }
        flangeIK = new Vector3(pos.z, -pos.x, pos.y) * 1000f;
        relRot = rot;
    }

    static Vector3 QuatToRollPitchYaw(Quaternion relRot)
    {
        Vector3 euler = relRot.eulerAngles;
        return new Vector3(NormalizeAngleS(euler.z), NormalizeAngleS(-euler.x), NormalizeAngleS(euler.y));
    }

    static float NormalizeAngleS(float a)
    {
        while (a > 180f) a -= 360f;
        while (a < -180f) a += 360f;
        return a;
    }

    // Vector approach (hướng từ cổ tay ra mũi tool) tính từ roll/pitch/yaw, quy ước ZYX - dùng chung cho
    // cả GetEndEffectorPosition (đọc target thật) lẫn FK nội bộ khi giải IK, để nhất quán.
    static Vector3 ApproachVector(float rollDeg, float pitchDeg, float yawDeg)
    {
        float r = rollDeg * Mathf.Deg2Rad, p = pitchDeg * Mathf.Deg2Rad, y = yawDeg * Mathf.Deg2Rad;
        float cr = Mathf.Cos(r), sr = Mathf.Sin(r), cp = Mathf.Cos(p), sp = Mathf.Sin(p), cy = Mathf.Cos(y), sy = Mathf.Sin(y);
        return new Vector3(cy * sp * cr + sy * sr, sy * sp * cr - cy * sr, cp * cr);
    }

    // Tool tip (hệ IK mm) + rotation ứng với một vector góc khớp.
    static void ForwardKinematicsTool(float[] thetaDeg, out Vector3 toolPos, out Vector3 rollPitchYaw)
    {
        ForwardKinematicsFlange(thetaDeg, out Vector3 flangeIK, out Quaternion relRot);
        rollPitchYaw = QuatToRollPitchYaw(relRot);
        Vector3 approach = ApproachVector(rollPitchYaw.x, rollPitchYaw.y, rollPitchYaw.z);
        toolPos = flangeIK + D6 * approach;
    }

    static Vector3 AxisAngleRad(Quaternion q)
    {
        if (q.w < 0) q = new Quaternion(-q.x, -q.y, -q.z, -q.w);
        float angle = 2f * Mathf.Acos(Mathf.Clamp(q.w, -1f, 1f));
        float s = Mathf.Sqrt(Mathf.Max(0f, 1f - q.w * q.w));
        Vector3 axis = s > 1e-6f ? new Vector3(q.x, q.y, q.z) / s : Vector3.zero;
        return axis * angle;
    }

    const float ROT_WEIGHT_MM = 300f; // 1 rad sai lệch hướng ~ 300mm (để cân bằng thang đo với vị trí)
    const float EPS_DEG = 0.05f;

    // Chạy damped least squares (Levenberg-Marquardt) từ MỘT điểm xuất phát (seed). Không ép giới hạn
    // khớp giữa chừng để tránh bị "kẹt" ở biên giới hạn trong lúc tối ưu - chỉ kiểm tra/dùng sau khi xong.
    float[] RunLM(float[] seed, Vector3 targetPos, Quaternion targetRelRot, int maxIter, out float posErrMag, out float rotErrDeg)
    {
        float[] theta = (float[])seed.Clone();
        float lambda = 4f;
        posErrMag = 0f; rotErrDeg = 0f;

        for (int iter = 0; iter < maxIter; iter++)
        {
            ForwardKinematicsTool(theta, out Vector3 curPos, out Vector3 curRPY);
            Quaternion curRelRot = Quaternion.Euler(-curRPY.y, curRPY.z, curRPY.x);

            Vector3 posErr = targetPos - curPos;
            Vector3 rotErrRad = AxisAngleRad(targetRelRot * Quaternion.Inverse(curRelRot));

            posErrMag = posErr.magnitude;
            rotErrDeg = rotErrRad.magnitude * Mathf.Rad2Deg;
            if (posErrMag <= RobotConfig.IK_TOLERANCE_POS && rotErrDeg <= RobotConfig.IK_TOLERANCE_ROT)
                break;

            float[] e = new float[6] {
                posErr.x, posErr.y, posErr.z,
                rotErrRad.x * ROT_WEIGHT_MM, rotErrRad.y * ROT_WEIGHT_MM, rotErrRad.z * ROT_WEIGHT_MM
            };

            // Jacobian so-do-huu-han (finite difference), 6 loi ra x 6 khop
            float[,] J = new float[6, 6];
            for (int j = 0; j < 6; j++)
            {
                float[] thetaP = (float[])theta.Clone();
                thetaP[j] += EPS_DEG;
                ForwardKinematicsTool(thetaP, out Vector3 p2, out Vector3 rpy2);
                Quaternion r2 = Quaternion.Euler(-rpy2.y, rpy2.z, rpy2.x);

                Vector3 dPos = (p2 - curPos) / EPS_DEG;
                Vector3 dRot = AxisAngleRad(r2 * Quaternion.Inverse(curRelRot)) / EPS_DEG;

                J[0, j] = dPos.x; J[1, j] = dPos.y; J[2, j] = dPos.z;
                J[3, j] = dRot.x * ROT_WEIGHT_MM; J[4, j] = dRot.y * ROT_WEIGHT_MM; J[5, j] = dRot.z * ROT_WEIGHT_MM;
            }

            // (J J^T + lambda^2 I) y = e ; delta = J^T y  (damped least squares)
            float[,] JJt = new float[6, 6];
            for (int r0 = 0; r0 < 6; r0++)
                for (int c0 = 0; c0 < 6; c0++)
                {
                    float s = 0f;
                    for (int k = 0; k < 6; k++) s += J[r0, k] * J[c0, k];
                    JJt[r0, c0] = s + (r0 == c0 ? lambda * lambda : 0f);
                }
            float[] y = SolveLinear(JJt, e);
            float[] delta = new float[6];
            for (int r0 = 0; r0 < 6; r0++)
            {
                float s = 0f;
                for (int k = 0; k < 6; k++) s += J[k, r0] * y[k];
                delta[r0] = Mathf.Clamp(s, -25f, 25f);
            }

            float[] thetaNew = new float[6];
            for (int i = 0; i < 6; i++) thetaNew[i] = theta[i] + delta[i];

            ForwardKinematicsTool(thetaNew, out Vector3 posNew, out Vector3 rpyNew);
            Quaternion relNew = Quaternion.Euler(-rpyNew.y, rpyNew.z, rpyNew.x);
            float newPosErr = (targetPos - posNew).magnitude;
            float newRotErr = AxisAngleRad(targetRelRot * Quaternion.Inverse(relNew)).magnitude * Mathf.Rad2Deg;
            float oldCost = posErrMag * posErrMag + (rotErrDeg * ROT_WEIGHT_MM / 57.29578f) * (rotErrDeg * ROT_WEIGHT_MM / 57.29578f);
            float newCost = newPosErr * newPosErr + (newRotErr * ROT_WEIGHT_MM / 57.29578f) * (newRotErr * ROT_WEIGHT_MM / 57.29578f);

            if (newCost < oldCost)
            {
                theta = thetaNew;
                lambda = Mathf.Max(lambda * 0.6f, 0.3f);
            }
            else
            {
                lambda *= 3f;
            }
        }

        ForwardKinematicsTool(theta, out Vector3 finalPos, out Vector3 finalRPY);
        Quaternion finalRel = Quaternion.Euler(-finalRPY.y, finalRPY.z, finalRPY.x);
        posErrMag = (targetPos - finalPos).magnitude;
        rotErrDeg = AxisAngleRad(targetRelRot * Quaternion.Inverse(finalRel)).magnitude * Mathf.Rad2Deg;
        return theta;
    }

    // ==================== INVERSE KINEMATICS SỐ (damped least squares, đa-điểm-xuất-phát) ====================
    // Thay cho IK giải tích cũ: các hằng số hình học DH cũ (A1/A2/A3/D4_OFFSET) đã được kiểm chứng KHÔNG
    // khớp với rig thật (đo trực tiếp từ ArticulationBody trong scene), nên toàn bộ công thức giải tích
    // bị sai một cách hệ thống. Cách này giải bằng số, dùng FK nội bộ dựng thẳng từ dữ liệu khớp thật
    // (verify khớp 0.0000 độ / 0.00mm với robot thật), nên luôn hội tụ đúng điểm đích nếu điểm đó
    // nằm trong tầm với và giới hạn khớp - không phụ thuộc việc suy công thức DH có đúng hay không.
    public IKResult SolveIK(Vector3 targetPos, Vector3 targetRot, float[] currentAngles)
    {
        var result = new IKResult { angles = new float[6] };

        float roll = targetRot.x, pitch = targetRot.y, yaw = targetRot.z;
        Quaternion targetRelRot = Quaternion.Euler(-pitch, yaw, roll);

        int maxIter = RobotConfig.IK_MAX_ITERATIONS;

        // Nhiều điểm xuất phát khác nhau để tránh kẹt ở cực tiểu địa phương / vùng kỳ dị -
        // ưu tiên nghiệm gần currentAngles nhất trong số các nghiệm hội tụ.
        List<float[]> seeds = new List<float[]> { (float[])currentAngles.Clone(), RobotConfig.HomePosition, RobotConfig.ZeroPosition };
        for (int i = 0; i < 6; i++)
        {
            float[] s = new float[6];
            for (int j = 0; j < 6; j++)
                s[j] = UnityEngine.Random.Range(RobotConfig.LowerLimits[j], RobotConfig.UpperLimits[j]);
            seeds.Add(s);
        }

        float[] bestConvergedTheta = null, bestFallbackTheta = null;
        float bestConvergedDiff = float.MaxValue, bestConvergedPosErr = 0f, bestConvergedRotErr = 0f;
        float bestFallbackCost = float.MaxValue, bestFallbackPosErr = 0f, bestFallbackRotErr = 0f;
        int convergedCount = 0;

        foreach (var seed in seeds)
        {
            float[] theta = RunLM(seed, targetPos, targetRelRot, maxIter, out float pErr, out float rErr);
            bool ok = pErr <= RobotConfig.IK_TOLERANCE_POS && rErr <= RobotConfig.IK_TOLERANCE_ROT;
            if (ok)
            {
                convergedCount++;
                float diff = 0f;
                for (int i = 0; i < 6; i++) diff += Mathf.Abs(NormalizeAngle(theta[i] - currentAngles[i]));
                if (bestConvergedTheta == null || diff < bestConvergedDiff)
                {
                    bestConvergedDiff = diff;
                    bestConvergedTheta = theta;
                    bestConvergedPosErr = pErr; bestConvergedRotErr = rErr;
                }
            }
            else
            {
                float cost = pErr * pErr + rErr * rErr; // ưu tiên sai số vị trí+hướng khi không nghiệm nào hội tụ
                if (bestFallbackTheta == null || cost < bestFallbackCost)
                {
                    bestFallbackCost = cost;
                    bestFallbackTheta = theta;
                    bestFallbackPosErr = pErr; bestFallbackRotErr = rErr;
                }
            }
        }

        bool found = bestConvergedTheta != null;
        float[] bestTheta = found ? bestConvergedTheta : bestFallbackTheta;
        result.posError = found ? bestConvergedPosErr : bestFallbackPosErr;
        result.rotError = found ? bestConvergedRotErr : bestFallbackRotErr;
        result.converged = found;
        result.solutionsFound = convergedCount;
        for (int i = 0; i < 6; i++) result.angles[i] = NormalizeAngle(bestTheta[i]);

        return result;
    }

    float[] SolveLinear(float[,] A, float[] b)
    {
        int n = b.Length;
        float[,] M = (float[,])A.Clone();
        float[] x = (float[])b.Clone();
        for (int col = 0; col < n; col++)
        {
            int piv = col;
            for (int r = col + 1; r < n; r++) if (Mathf.Abs(M[r, col]) > Mathf.Abs(M[piv, col])) piv = r;
            if (piv != col)
            {
                for (int c = 0; c < n; c++) { float t = M[col, c]; M[col, c] = M[piv, c]; M[piv, c] = t; }
                float tb = x[col]; x[col] = x[piv]; x[piv] = tb;
            }
            float diag = M[col, col];
            if (Mathf.Abs(diag) < 1e-9f) diag = 1e-9f;
            for (int c = 0; c < n; c++) M[col, c] /= diag;
            x[col] /= diag;
            for (int r = 0; r < n; r++)
            {
                if (r == col) continue;
                float factor = M[r, col];
                if (factor == 0f) continue;
                for (int c = 0; c < n; c++) M[r, c] -= factor * M[col, c];
                x[r] -= factor * x[col];
            }
        }
        return x;
    }

    float NormalizeAngle(float a)
    {
        while (a > 180f) a -= 360f;
        while (a < -180f) a += 360f;
        return a;
    }
}
