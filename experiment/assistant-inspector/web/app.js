(() => {
    const state = { category: "餐桌", replay: "fresh", tab: "full", observation: "obs_001", vlm: null, navigationRuns: [] };
    const $ = (selector) => document.querySelector(selector);
    const escapeHtml = (value) => String(value).replace(/[&<>"']/g, (character) => ({"&":"&amp;", "<":"&lt;", ">":"&gt;", "\"":"&quot;", "'":"&#039;"}[character]));

    function renderRoutes(routes) {
        $("#route-table").innerHTML = routes.map((route) => `<tr><td>${escapeHtml(route.id)}</td><td class="ok">${route.status} ${escapeHtml(route.status_name)}</td><td>${route.localization_goal_error_m.toFixed(3)} m</td><td>${route.localization_truth_error_max_m.toFixed(3)} m</td></tr>`).join("");
    }

    function renderCandidates(candidates) {
        const list = $("#candidate-list");
        if (!candidates.length) { list.innerHTML = ""; return; }
        list.innerHTML = candidates.map((candidate) => `<article class="candidate"><img src="${candidate.evidence.image_url}" alt="${escapeHtml(candidate.candidate_id)} 的公开 RGB 证据"><div><code>${escapeHtml(candidate.candidate_id)}</code><p>${escapeHtml(candidate.label)} · 置信度 ${candidate.detector_confidence.toFixed(3)}</p><p>${escapeHtml(candidate.geometry.status)}${candidate.geometry.depth_median_m ? ` · 深度中位数 ${candidate.geometry.depth_median_m.toFixed(2)} m` : ""}</p></div></article>`).join("");
    }

    async function queryMemory() {
        const button = $("#query-button");
        button.disabled = true;
        $("#result-status").textContent = "查询中";
        $("#result-reason").textContent = "正在读取本地持久化 store。";
        try {
            const response = await fetch(`/api/query?category=${encodeURIComponent(state.category)}&replay=${state.replay}`);
            const result = await response.json();
            if (!response.ok) throw new Error(result.error || "查询失败");
            $("#result-status").textContent = result.status;
            $("#motion-state").textContent = `motion_authorized: ${result.motion_authorized}`;
            const candidateIds = result.candidate_ids.length ? `候选：${result.candidate_ids.join(", ")}；` : "";
            $("#result-reason").textContent = `${result.reason}；${candidateIds}回放 ${result.replay} (${result.replay_time_ns} ns)。`;
            renderCandidates(result.candidates);
        } catch (error) {
            $("#result-status").textContent = "错误";
            $("#result-reason").textContent = error.message;
            $("#candidate-list").innerHTML = "";
        } finally { button.disabled = false; }
    }

    function renderVlm() {
        const record = state.vlm.records.find((item) => item.observation_id === state.observation);
        const output = record.outputs[state.tab];
        $("#vlm-image").src = record.images[state.tab];
        $("#vlm-image").alt = `${record.observation_id} ${state.tab} 保存证据`;
        $("#vlm-file").textContent = record.images[state.tab].split("/").pop();
        $("#vlm-output").innerHTML = [
            ["raw", JSON.stringify(output.raw)], ["strict_format_valid", output.strict_format_valid],
            ["finite_label_adapter_valid", output.finite_label_adapter_valid], ["adapter_label", output.adapter_label],
            ["selected_id", output.selected_id], ["latency_seconds", `${output.latency_seconds.toFixed(3)} s`]
        ].map(([name, value]) => `<dt>${name}</dt><dd>${escapeHtml(value)}</dd>`).join("");
        $("#vlm-candidates").textContent = `候选 ID：${record.candidate_ids.join(", ")}`;
        $("#vlm-budget").textContent = `调用预算：${state.vlm.summary.generation_calls} / ${state.vlm.summary.generation_budget}；保存结果不重新推理。`;
    }

    function selectButton(container, attribute, value) {
        container.querySelectorAll("button").forEach((button) => button.classList.toggle("selected", button.dataset[attribute] === value));
    }

    async function initialize() {
        try {
            const [overviewResponse, vlmResponse] = await Promise.all([fetch("/api/overview"), fetch("/api/vlm")]);
            const overview = await overviewResponse.json();
            state.vlm = await vlmResponse.json();
            if (!overviewResponse.ok || !vlmResponse.ok) throw new Error("初始化证据读取失败");
            $("#record-count").textContent = overview.semantic_memory.import_result.records;
            state.navigationRuns = overview.navigation_runs;
            $("#navigation-run-select").innerHTML = state.navigationRuns.map((run, index) => `<option value="${index}">${escapeHtml(run.label)}</option>`).join("");
            $("#nav-source").textContent = state.navigationRuns[0].source;
            renderRoutes(state.navigationRuns[0].routes);
            $("#strict-count").textContent = `${state.vlm.summary.strict_format_valid}/16`;
            $("#adapter-count").textContent = `${state.vlm.summary.finite_label_adapter_valid}/16`;
            $("#reversal-count").textContent = `${state.vlm.summary.reversal_flips}/${state.vlm.summary.reversal_total}`;
            $("#observation-select").innerHTML = state.vlm.records.map((record) => `<option value="${record.observation_id}">${record.observation_id}</option>`).join("");
            renderVlm();
            queryMemory();
        } catch (error) {
            $("#result-status").textContent = "初始化错误";
            $("#result-reason").textContent = error.message;
        }
    }

    $("#category-control").addEventListener("click", (event) => { const value = event.target.dataset.category; if (!value) return; state.category = value; selectButton(event.currentTarget, "category", value); });
    $("#replay-control").addEventListener("click", (event) => { const value = event.target.dataset.replay; if (!value) return; state.replay = value; selectButton(event.currentTarget, "replay", value); });
    $("#navigation-run-select").addEventListener("change", (event) => {
        const run = state.navigationRuns[Number(event.target.value)];
        if (!run) return;
        $("#nav-source").textContent = run.source;
        renderRoutes(run.routes);
    });
    $("#query-button").addEventListener("click", queryMemory);
    $("#observation-select").addEventListener("change", (event) => { state.observation = event.target.value; renderVlm(); });
    $(".tab-list").addEventListener("click", (event) => { const value = event.target.dataset.tab; if (!value) return; state.tab = value; selectButton(event.currentTarget, "tab", value); renderVlm(); });
    initialize();
})();
