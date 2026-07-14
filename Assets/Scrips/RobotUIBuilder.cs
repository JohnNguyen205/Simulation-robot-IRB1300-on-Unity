using UnityEngine;
using UnityEngine.UI;
using TMPro;

public class RobotUIBuilder
{
    public Slider[] jointSliders = new Slider[6];
    public TMP_Text[] jointValueTexts = new TMP_Text[6];
    public TMP_Text[] jointIKTexts = new TMP_Text[6];   // ⭐ MỚI: hiển thị góc IK tính ra
    public TMP_Text pxText, pyText, pzText;
    public TMP_Text rxText, ryText, rzText;
    public TMP_InputField ikXInput, ikYInput, ikZInput;
    public TMP_InputField ikRollInput, ikPitchInput, ikYawInput;
    public TMP_Text ikStatusText;
    public Button goToButton, resetButton, homeButton;

    Transform panel;

    public void BuildAll(Transform canvas)
    {
        panel = CreatePanel(canvas);
        CreateTitle();
        BuildSliders();
        BuildWorkspaceInfo();
        BuildFKSection();
        BuildIKSection();
        BuildButtons();
    }

    Transform CreatePanel(Transform canvas)
    {
        if (canvas.Find("ControlPanel") != null) return canvas.Find("ControlPanel");
        GameObject p = new GameObject("ControlPanel", typeof(RectTransform));
        p.transform.SetParent(canvas, false);
        var rt = p.GetComponent<RectTransform>();
        rt.anchorMin = new Vector2(0, 1); rt.anchorMax = new Vector2(0, 1);
        rt.pivot = new Vector2(0, 1); rt.anchoredPosition = new Vector2(25, -25);
        rt.sizeDelta = new Vector2(780, 1030);   // ⭐ To hơn chút để chứa cột IK
        p.transform.localScale = Vector3.one * 0.62f; // thu nho toan bo panel cho gon
        var img = p.AddComponent<Image>();
        img.color = new Color(0.10f, 0.09f, 0.09f, 0.90f);
        return p.transform;
    }

    void CreateTitle()
    {
        MakeText(panel, "Title", "ABB IRB1300 CONTROL PANEL",
                 new Vector2(15, -20), new Vector2(750, 55),
                 32, Color.white, TextAlignmentOptions.Center, true);

        // Header nhỏ cho cột góc hiện tại và cột IK
        MakeText(panel, "H_Cur", "Hien tai",
                 new Vector2(520, -95), new Vector2(90, 20),
                 12, new Color(0.8f, 0.8f, 0.8f), TextAlignmentOptions.Left, true);
        MakeText(panel, "H_IK", "IK tinh",
                 new Vector2(625, -95), new Vector2(90, 20),
                 12, new Color(0.4f, 0.85f, 1f), TextAlignmentOptions.Left, true);
    }

    void BuildSliders()
    {
        Transform canvas = GameObject.Find("Canvas").transform;
        for (int i = 0; i < 6; i++)
        {
            GameObject sGO = GameObject.Find(RobotConfig.SliderNames[i]);
            if (sGO == null) sGO = CreateSlider(RobotConfig.SliderNames[i], canvas);

            sGO.transform.SetParent(panel, false);
            var srt = sGO.GetComponent<RectTransform>();
            srt.anchorMin = new Vector2(0, 1); srt.anchorMax = new Vector2(0, 1);
            srt.pivot = new Vector2(0, 1);
            srt.anchoredPosition = new Vector2(130, -155 - i * 55);
            srt.sizeDelta = new Vector2(360, 24);   // Slider ngắn hơn chút

            var s = sGO.GetComponent<Slider>();
            s.minValue = RobotConfig.LowerLimits[i];
            s.maxValue = RobotConfig.UpperLimits[i];
            s.value = 0f;
            jointSliders[i] = s;

            // Cột 1: góc hiện tại (trắng)
            jointValueTexts[i] = MakeText(sGO.transform, $"Value_{i + 1}", "0.0°",
                new Vector2(10, 0), new Vector2(90, 28),
                16, Color.white, TextAlignmentOptions.Left, false,
                anchor: new Vector2(1, 0.5f), pivot: new Vector2(0, 0.5f));

            // ⭐ Cột 2: góc IK tính ra (xanh dương)
            jointIKTexts[i] = MakeText(sGO.transform, $"IK_{i + 1}", "",
                new Vector2(110, 0), new Vector2(100, 28),
                15, new Color(0.4f, 0.85f, 1f), TextAlignmentOptions.Left, false,
                anchor: new Vector2(1, 0.5f), pivot: new Vector2(0, 0.5f));

            MakeText(sGO.transform, "Label", $"Theta {i + 1}",
                new Vector2(-10, 0), new Vector2(100, 28),
                17, Color.white, TextAlignmentOptions.Right, true,
                anchor: new Vector2(0, 0.5f), pivot: new Vector2(1, 0.5f));
        }
    }

    GameObject CreateSlider(string name, Transform canvas)
    {
        GameObject sliderGO = new GameObject(name, typeof(RectTransform));
        sliderGO.transform.SetParent(canvas, false);
        var slider = sliderGO.AddComponent<Slider>();

        GameObject bg = new GameObject("Background", typeof(RectTransform));
        bg.transform.SetParent(sliderGO.transform, false);
        var bgRT = bg.GetComponent<RectTransform>();
        bgRT.anchorMin = new Vector2(0, 0.25f); bgRT.anchorMax = new Vector2(1, 0.75f);
        bgRT.sizeDelta = Vector2.zero;
        var bgImg = bg.AddComponent<Image>();
        bgImg.color = new Color(0.3f, 0.3f, 0.3f);

        GameObject fillArea = new GameObject("Fill Area", typeof(RectTransform));
        fillArea.transform.SetParent(sliderGO.transform, false);
        var faRT = fillArea.GetComponent<RectTransform>();
        faRT.anchorMin = new Vector2(0, 0.25f); faRT.anchorMax = new Vector2(1, 0.75f);
        faRT.offsetMin = new Vector2(5, 0); faRT.offsetMax = new Vector2(-15, 0);

        GameObject fill = new GameObject("Fill", typeof(RectTransform));
        fill.transform.SetParent(fillArea.transform, false);
        var fillRT = fill.GetComponent<RectTransform>();
        fillRT.sizeDelta = new Vector2(10, 0);
        var fillImg = fill.AddComponent<Image>();
        fillImg.color = new Color(1f, 0.6f, 0.2f);
        slider.fillRect = fillRT;

        GameObject handleArea = new GameObject("Handle Slide Area", typeof(RectTransform));
        handleArea.transform.SetParent(sliderGO.transform, false);
        var haRT = handleArea.GetComponent<RectTransform>();
        haRT.anchorMin = Vector2.zero; haRT.anchorMax = Vector2.one;
        haRT.offsetMin = new Vector2(10, 0); haRT.offsetMax = new Vector2(-10, 0);

        GameObject handle = new GameObject("Handle", typeof(RectTransform));
        handle.transform.SetParent(handleArea.transform, false);
        var hRT = handle.GetComponent<RectTransform>();
        hRT.sizeDelta = new Vector2(20, 20);
        var hImg = handle.AddComponent<Image>();
        hImg.color = Color.white;
        slider.handleRect = hRT;
        slider.targetGraphic = hImg;
        slider.direction = Slider.Direction.LeftToRight;
        return sliderGO;
    }

    void BuildWorkspaceInfo()
    {
        MakeSection("WORKSPACE (VUNG LAM VIEC)", new Vector2(25, -490));
        MakeInfoRow("Reach:",
            $"{RobotConfig.REACH_MIN:F0} - {RobotConfig.REACH_MAX:F0} mm (ban kinh)",
            new Vector2(25, -520));
        MakeInfoRow("Height:",
            $"{RobotConfig.Z_MIN:F0} - {RobotConfig.Z_MAX:F0} mm (tu de)",
            new Vector2(25, -548));
    }

    void BuildFKSection()
    {
        MakeSection("FORWARD KINEMATICS (VI TRI + HUONG)", new Vector2(25, -585));
        pxText = MakeFKRow("Px", new Vector2(25, -615), "mm");
        pyText = MakeFKRow("Py", new Vector2(25, -645), "mm");
        pzText = MakeFKRow("Pz", new Vector2(25, -675), "mm");
        rxText = MakeFKRow("Roll", new Vector2(400, -615), "°", 60);
        ryText = MakeFKRow("Pitch", new Vector2(400, -645), "°", 60);
        rzText = MakeFKRow("Yaw", new Vector2(400, -675), "°", 60);
    }

    void BuildIKSection()
    {
        MakeSection("INVERSE KINEMATICS (NHAP TOA DO + HUONG DICH)", new Vector2(25, -720));
        ikXInput = MakeInput("X", new Vector2(25, -755), "500", new Color(1f, 0.85f, 0.4f));
        ikYInput = MakeInput("Y", new Vector2(275, -755), "0", new Color(1f, 0.85f, 0.4f));
        ikZInput = MakeInput("Z", new Vector2(525, -755), "900", new Color(1f, 0.85f, 0.4f));
        ikRollInput = MakeInput("R", new Vector2(25, -805), "0", new Color(0.7f, 0.9f, 1f), "Roll");
        ikPitchInput = MakeInput("P", new Vector2(275, -805), "0", new Color(0.7f, 0.9f, 1f), "Pitch");
        ikYawInput = MakeInput("Y", new Vector2(525, -805), "0", new Color(0.7f, 0.9f, 1f), "Yaw");

        ikStatusText = MakeText(panel, "IK_Status",
            "Nhap X,Y,Z (mm) va Roll,Pitch,Yaw (do) roi bam GO TO",
            new Vector2(25, -848), new Vector2(730, 25),
            14, new Color(0.7f, 0.7f, 0.7f), TextAlignmentOptions.Left, false);
        ikStatusText.richText = true;
    }

    void BuildButtons()
    {
        goToButton = MakeButton("GO TO POSITION", new Vector2(25, -880), new Color(0.2f, 0.55f, 0.3f), 730);
        resetButton = MakeButton("RESET", new Vector2(25, -950), new Color(0.55f, 0.25f, 0.25f), 355);
        homeButton = MakeButton("HOME", new Vector2(400, -950), new Color(0.25f, 0.45f, 0.55f), 355);
    }

    TMP_Text MakeText(Transform parent, string name, string content,
        Vector2 pos, Vector2 size, float fontSize, Color color,
        TextAlignmentOptions align, bool bold,
        Vector2? anchor = null, Vector2? pivot = null)
    {
        GameObject go = new GameObject(name, typeof(RectTransform));
        go.transform.SetParent(parent, false);
        var rt = go.GetComponent<RectTransform>();
        Vector2 a = anchor ?? new Vector2(0, 1);
        Vector2 pv = pivot ?? new Vector2(0, 1);
        rt.anchorMin = a; rt.anchorMax = a; rt.pivot = pv;
        rt.anchoredPosition = pos; rt.sizeDelta = size;
        var t = go.AddComponent<TextMeshProUGUI>();
        t.text = content; t.fontSize = fontSize; t.color = color;
        t.alignment = align;
        if (bold) t.fontStyle = FontStyles.Bold;
        return t;
    }

    void MakeSection(string content, Vector2 pos)
    {
        MakeText(panel, "Section", content, pos, new Vector2(730, 28),
                 16, new Color(0.85f, 0.75f, 0.6f), TextAlignmentOptions.Left, true);
    }

    void MakeInfoRow(string label, string value, Vector2 pos)
    {
        MakeText(panel, "InfoL", label, pos, new Vector2(80, 25),
                 14, new Color(0.7f, 0.85f, 1f), TextAlignmentOptions.Left, true);
        MakeText(panel, "InfoV", value, new Vector2(pos.x + 80, pos.y), new Vector2(560, 25),
                 14, new Color(0.85f, 0.85f, 0.85f), TextAlignmentOptions.Left, false);
    }

    TMP_Text MakeFKRow(string label, Vector2 pos, string unit, float labelW = 65)
    {
        MakeText(panel, $"L_{label}", label, pos, new Vector2(labelW, 28),
                 15, Color.white, TextAlignmentOptions.Left, true);
        return MakeText(panel, $"V_{label}", "0.00 " + unit,
                 new Vector2(pos.x + labelW, pos.y), new Vector2(300, 28),
                 15, new Color(1f, 0.85f, 0.4f), TextAlignmentOptions.Left, false);
    }

    TMP_InputField MakeInput(string label, Vector2 pos, string defVal, Color labelColor, string customLabel = null)
    {
        MakeText(panel, $"IK_L_{label}", customLabel ?? label,
                 pos, new Vector2(50, 30),
                 16, labelColor, TextAlignmentOptions.Left, true);

        GameObject inp = new GameObject($"IK_I_{label}", typeof(RectTransform));
        inp.transform.SetParent(panel, false);
        var irt = inp.GetComponent<RectTransform>();
        irt.anchorMin = new Vector2(0, 1); irt.anchorMax = new Vector2(0, 1);
        irt.pivot = new Vector2(0, 1);
        irt.anchoredPosition = new Vector2(pos.x + 55, pos.y);
        irt.sizeDelta = new Vector2(180, 35);
        var bg = inp.AddComponent<Image>();
        bg.color = new Color(0.22f, 0.22f, 0.25f);
        var field = inp.AddComponent<TMP_InputField>();

        GameObject ta = new GameObject("TA", typeof(RectTransform), typeof(RectMask2D));
        ta.transform.SetParent(inp.transform, false);
        var tart = ta.GetComponent<RectTransform>();
        tart.anchorMin = Vector2.zero; tart.anchorMax = Vector2.one;
        tart.offsetMin = new Vector2(10, 5); tart.offsetMax = new Vector2(-10, -5);

        GameObject txtGO = new GameObject("T", typeof(RectTransform));
        txtGO.transform.SetParent(ta.transform, false);
        var trt = txtGO.GetComponent<RectTransform>();
        trt.anchorMin = Vector2.zero; trt.anchorMax = Vector2.one;
        trt.sizeDelta = Vector2.zero;
        var txt = txtGO.AddComponent<TextMeshProUGUI>();
        txt.fontSize = 15; txt.color = Color.white;
        txt.alignment = TextAlignmentOptions.MidlineLeft;

        field.textViewport = tart; field.textComponent = txt;
        field.text = defVal;
        field.lineType = TMP_InputField.LineType.SingleLine;
        return field;
    }

    Button MakeButton(string label, Vector2 pos, Color color, float width)
    {
        GameObject go = new GameObject(label + "Btn", typeof(RectTransform));
        go.transform.SetParent(panel, false);
        var rt = go.GetComponent<RectTransform>();
        rt.anchorMin = new Vector2(0, 1); rt.anchorMax = new Vector2(0, 1);
        rt.pivot = new Vector2(0, 1); rt.anchoredPosition = pos;
        rt.sizeDelta = new Vector2(width, 55);
        var img = go.AddComponent<Image>(); img.color = color;
        var b = go.AddComponent<Button>(); b.targetGraphic = img;

        GameObject txtGO = new GameObject("T", typeof(RectTransform));
        txtGO.transform.SetParent(go.transform, false);
        var trt = txtGO.GetComponent<RectTransform>();
        trt.anchorMin = Vector2.zero; trt.anchorMax = Vector2.one;
        trt.sizeDelta = Vector2.zero;
        trt.anchoredPosition = Vector2.zero;
        var t = txtGO.AddComponent<TextMeshProUGUI>();
        t.text = label; t.fontSize = 22; t.color = Color.white;
        t.alignment = TextAlignmentOptions.Center; t.fontStyle = FontStyles.Bold;
        return b;
    }
}