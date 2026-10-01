from __future__ import annotations

from concurrent.futures import ThreadPoolExecutor
import json
from pathlib import Path
import tkinter as tk
from tkinter import filedialog, messagebox, ttk

from core import (ACTIVE_SLOTS, LIMBS, AuthoringError, EditOptions, NativeTool, SupportInterval,
                  export_override, generate_contacts, inspect_hkx, load_template)


class AuthoringApp:
    def __init__(self, window):
        self.window = window
        self.window.title("FreeClimb 动作制作助手")
        self.window.geometry("980x820")
        self.window.minsize(850, 680)
        self.executor = ThreadPoolExecutor(max_workers=1)
        self.future = None
        self.after_success = None
        self.pack = None
        self.inspection = None
        self.inspected_path = None
        self.intervals = []
        self.buttons = []
        self.path_vars = {key: tk.StringVar() for key in ("pack", "hkx", "output")}
        self.slot = tk.StringVar(value="up")
        self.contact_mode = tk.StringVar(value="template")
        self.advanced_enabled = tk.BooleanVar(value=False)
        self.basic_vars = {key: tk.StringVar() for key in ("stride", "height", "x", "y", "z")}
        self.limb = tk.StringVar(value=LIMBS[0])
        self.start = tk.StringVar(value="0")
        self.end = tk.StringVar(value="1")
        self.fade = tk.StringVar(value="0.06")
        self.status = tk.StringVar(value="先选择完整基础包的 pack.json；原文件不会被修改。")
        outer = ttk.Frame(window, padding=12)
        outer.pack(fill="both", expand=True)
        outer.columnconfigure(1, weight=1)
        self.path_row(outer, 0, "基础动作包", "pack", self.choose_pack)
        ttk.Label(outer, text="替换动作槽").grid(row=1, column=0, sticky="w", pady=4)
        self.slot_box = ttk.Combobox(outer, textvariable=self.slot, values=ACTIVE_SLOTS, state="readonly")
        self.slot_box.grid(row=1, column=1, sticky="ew", padx=8)
        self.slot_box.bind("<<ComboboxSelected>>", lambda event: self.show_template())
        self.path_row(outer, 2, "已适配 HKX", "hkx", self.choose_hkx)
        self.path_row(outer, 3, "输出覆盖 ZIP", "output", self.choose_output)
        inspect_button = ttk.Button(outer, text="检查 HKX", command=self.inspect)
        inspect_button.grid(row=4, column=2, pady=4)
        self.buttons.append(inspect_button)
        self.metadata = ttk.Label(outer, text="时长、帧数和轨道数：尚未检查", wraplength=760)
        self.metadata.grid(row=4, column=0, columnspan=2, sticky="w", pady=4)
        ttk.Label(outer, text="本工具不做骨架重定向，也不会自动识别抓放手。原生校验通过后才会导出。",
                  wraplength=900).grid(row=5, column=0, columnspan=3, sticky="w", pady=(4, 10))
        notebook = ttk.Notebook(outer)
        notebook.grid(row=6, column=0, columnspan=3, sticky="nsew")
        outer.rowconfigure(6, weight=1)
        self.contact_page(notebook)
        self.advanced_page(notebook)
        footer = ttk.Frame(outer)
        footer.grid(row=7, column=0, columnspan=3, sticky="ew", pady=(10, 0))
        footer.columnconfigure(0, weight=1)
        ttk.Label(footer, textvariable=self.status, wraplength=700).grid(row=0, column=0, sticky="w")
        button = ttk.Button(footer, text="校验并导出 MO2 覆盖包", command=self.export)
        button.grid(row=0, column=1, padx=(8, 0))
        self.buttons.append(button)
        self.details = tk.Text(outer, height=4, wrap="word", state="disabled")
        self.details.grid(row=8, column=0, columnspan=3, sticky="ew", pady=(8, 0))
        self.window.protocol("WM_DELETE_WINDOW", self.close)
        self.window.after(100, self.poll)

    def path_row(self, parent, row, title, key, callback):
        ttk.Label(parent, text=title).grid(row=row, column=0, sticky="w", pady=4)
        ttk.Entry(parent, textvariable=self.path_vars[key]).grid(row=row, column=1, sticky="ew", padx=8)
        button = ttk.Button(parent, text="浏览…", command=callback)
        button.grid(row=row, column=2)
        self.buttons.append(button)

    def contact_page(self, notebook):
        page = ttk.Frame(notebook, padding=10)
        notebook.add(page, text="接触标记与基本参数")
        ttk.Radiobutton(page, text="保留模板的接触曲线（按归一化相位用于新动作）",
                        variable=self.contact_mode, value="template").pack(anchor="w")
        ttk.Radiobutton(page, text="手工标记支撑区间，生成四肢接触曲线",
                        variable=self.contact_mode, value="manual").pack(anchor="w", pady=(4, 8))
        ttk.Label(page, text="区间外权重为 0；区间内从 0 渐入至 1，再渐出至 0。过渡包含在区间内；同一肢体重叠区间取较大值。未标记的肢体不支撑。",
                  wraplength=870).pack(anchor="w", pady=(0, 8))
        bar = ttk.Frame(page)
        bar.pack(fill="x")
        ttk.Combobox(bar, textvariable=self.limb, values=LIMBS, width=7, state="readonly").pack(side="left")
        for label, variable in (("开始秒", self.start), ("结束秒", self.end), ("过渡秒", self.fade)):
            ttk.Label(bar, text=label).pack(side="left", padx=(10, 4))
            ttk.Entry(bar, textvariable=variable, width=7).pack(side="left")
        for label, command in (("添加", self.add_interval), ("删除选中", self.remove_interval)):
            button = ttk.Button(bar, text=label, command=command)
            button.pack(side="left", padx=(8, 0))
            self.buttons.append(button)
        columns = ("limb", "start", "end", "fade")
        table_frame = ttk.Frame(page)
        table_frame.pack(fill="both", expand=True, pady=10)
        self.table = ttk.Treeview(table_frame, columns=columns, show="headings", height=7)
        for column, title in zip(columns, ("肢体", "支撑开始（秒）", "支撑结束（秒）", "柔和过渡（秒）")):
            self.table.heading(column, text=title)
            self.table.column(column, width=170, stretch=True)
        scrollbar = ttk.Scrollbar(table_frame, orient="vertical", command=self.table.yview)
        self.table.configure(yscrollcommand=scrollbar.set)
        scrollbar.pack(side="right", fill="y")
        self.table.pack(side="left", fill="both", expand=True)
        basic = ttk.LabelFrame(page, text="可选参数：留空保留模板值；单位为 Skyrim 游戏单位", padding=8)
        basic.pack(fill="x")
        for index, (name, title) in enumerate((("stride", "步幅 stride"), ("height", "抬升 height"),
                                              ("x", "travel X"), ("y", "travel Y"), ("z", "travel Z"))):
            ttk.Label(basic, text=title).grid(row=0, column=index, padx=5, sticky="w")
            ttk.Entry(basic, textvariable=self.basic_vars[name], width=13).grid(row=1, column=index, padx=5)
        self.template_values = ttk.Label(page, text="模板参数：尚未读取", wraplength=860)
        self.template_values.pack(anchor="w", pady=(8, 0))

    def advanced_page(self, notebook):
        page = ttk.Frame(notebook, padding=10)
        notebook.add(page, text="高级配置与特殊动作")
        ttk.Label(page, text="情境侧跃与登顶有配套路径和抓放窗口。仅标记 contacts 不会同步更改这些窗口；替换动作必须与它们匹配。",
                  wraplength=870).pack(anchor="w")
        self.special = ttk.Label(page, text="选择动作槽后显示专用参数。", wraplength=870)
        self.special.pack(anchor="w", pady=8)
        ttk.Checkbutton(page, text="使用下方高级 JSON（slot / file 不可改；基本参数与手工接触模式随后覆盖相应字段）",
                        variable=self.advanced_enabled).pack(anchor="w", pady=(0, 8))
        frame = ttk.Frame(page)
        frame.pack(fill="both", expand=True)
        self.advanced = tk.Text(frame, wrap="none", undo=True, height=17)
        ybar = ttk.Scrollbar(frame, orient="vertical", command=self.advanced.yview)
        xbar = ttk.Scrollbar(frame, orient="horizontal", command=self.advanced.xview)
        self.advanced.configure(yscrollcommand=ybar.set, xscrollcommand=xbar.set)
        ybar.pack(side="right", fill="y")
        xbar.pack(side="bottom", fill="x")
        self.advanced.pack(fill="both", expand=True)

    def choose_pack(self):
        path = filedialog.askopenfilename(title="选择完整基础包的 pack.json", filetypes=[("动作包清单", "pack.json")])
        if path:
            self.path_vars["pack"].set(path)
            self.run("正在读取完整基础包…", lambda: load_template(path), self.loaded_pack)

    def loaded_pack(self, pack):
        self.pack = pack
        self.slot_box.configure(values=tuple(pack.configurations))
        if self.slot.get() not in pack.configurations:
            self.slot.set("up")
        self.show_template()
        self.status.set(f"已读取 {len(pack.configurations)} 个槽的模板。导出前会使用原生组件校验完整候选包。")

    def show_template(self):
        if self.pack is None or self.slot.get() not in self.pack.configurations:
            return
        config = self.pack.configurations[self.slot.get()]
        self.advanced.delete("1.0", "end")
        self.advanced.insert("1.0", json.dumps(config, ensure_ascii=False, indent=2))
        self.advanced_enabled.set(False)
        for variable in self.basic_vars.values():
            variable.set("")
        self.contact_mode.set("template")
        self.intervals.clear()
        self.refresh_intervals()
        self.template_values.configure(text=f"模板参数：stride={config.get('stride')}；height={config.get('height')}；travel={config.get('travel')}")
        keys = ("path", "sourceHands", "targetHands", "verticalBlend", "unplant", "replant", "releaseHands", "replantSamplePhase", "returnWindow")
        special = {key: config[key] for key in keys if key in config}
        summary = "；".join(f"{key}={len(value)} 个节点（完整内容见下方）" if key == "path" else f"{key}={value}"
                            for key, value in special.items())
        self.special.configure(text=summary or ("contextHang 的首帧手脚位置参与侧跃校准，请连同左右侧跃检查。" if self.slot.get() == "contextHang" else "该槽没有上述专用路径或抓放窗口。"))

    def choose_hkx(self):
        path = filedialog.askopenfilename(title="选择已适配的 HKX", filetypes=[("HKX 动画", "*.hkx")])
        if path:
            self.path_vars["hkx"].set(path)
            self.inspect()

    def inspect(self):
        path = self.path_vars["hkx"].get().strip()
        if self.intervals and not messagebox.askyesno("重新读取动画", "重新检查 HKX 会清空已有支撑标记，避免把上一个动画的时间套到新动作。继续吗？"):
            if self.inspected_path is not None:
                self.path_vars["hkx"].set(str(self.inspected_path))
            return
        self.inspection = None
        self.inspected_path = None

        def complete(result):
            self.inspection = result
            self.inspected_path = Path(path).resolve()
            self.end.set(repr(result["duration"]))
            self.intervals.clear()
            self.refresh_intervals()
            self.contact_mode.set("template")
            self.metadata.configure(text=f"时长 {result['duration']:.6g} 秒；{result['frames']} 帧；{result['tracks']} 条轨道。此检查不等同于完整包适配通过。")
            self.status.set("HKX 已读取，可标记支撑区间。")
        self.run("正在读取 HKX…", lambda: inspect_hkx(path), complete)

    def choose_output(self):
        path = filedialog.asksaveasfilename(title="保存 MO2 覆盖包", defaultextension=".zip",
                                          initialfile=f"FreeClimb-{self.slot.get()}-override.zip", filetypes=[("ZIP 覆盖包", "*.zip")])
        if path:
            self.path_vars["output"].set(path)

    def add_interval(self):
        try:
            if self.inspection is None or self.inspected_path != Path(self.path_vars["hkx"].get()).resolve():
                raise AuthoringError("请先检查当前 HKX，取得正确时长。")
            interval = SupportInterval(LIMBS.index(self.limb.get()), float(self.start.get()), float(self.end.get()), float(self.fade.get()))
            generate_contacts(self.inspection["duration"], self.intervals + [interval], samples=self.inspection["frames"])
            self.intervals.append(interval)
            self.contact_mode.set("manual")
            self.refresh_intervals()
        except (ValueError, OSError) as error:
            self.failure(error)

    def refresh_intervals(self):
        self.table.delete(*self.table.get_children())
        for index, interval in enumerate(self.intervals):
            self.table.insert("", "end", iid=str(index), values=(LIMBS[interval.limb], interval.start, interval.end, interval.fade))

    def remove_interval(self):
        selected = {int(i) for i in self.table.selection()}
        self.intervals = [value for index, value in enumerate(self.intervals) if index not in selected]
        self.refresh_intervals()

    def export(self):
        try:
            paths = {key: value.get().strip() for key, value in self.path_vars.items()}
            if not all(paths.values()):
                raise AuthoringError("请先选择基础包、HKX 和输出 ZIP。")
            if self.pack is None or Path(paths["pack"]).resolve() != self.pack.path:
                raise AuthoringError("基础包路径改变后，请用“浏览”重新读取模板。")
            if self.contact_mode.get() == "manual" and (self.inspection is None or self.inspected_path != Path(paths["hkx"]).resolve()):
                raise AuthoringError("手工接触模式需要先检查当前 HKX，再为该动画标记支撑区间。")
            if Path(paths["output"]).exists() and not messagebox.askyesno("覆盖已有 ZIP", "校验成功后将替换这个 ZIP。继续吗？"):
                return
            values = {key: (float(value.get()) if value.get().strip() else None) for key, value in self.basic_vars.items()}
            options = EditOptions(self.contact_mode.get(), list(self.intervals), values["stride"], values["height"],
                                  (values["x"], values["y"], values["z"]),
                                  self.advanced.get("1.0", "end-1c") if self.advanced_enabled.get() else None)
            slot = self.slot.get()
            self.run("正在创建完整候选包并校验，原基础包不会改变…",
                     lambda: export_override(paths["pack"], slot, paths["hkx"], paths["output"], options), self.exported)
        except (ValueError, OSError) as error:
            self.failure(error)

    def exported(self, result):
        self.status.set(f"导出成功：{result['slot']}，共 2 个覆盖文件。")
        self.set_details(result["output"] + "\n" + "\n".join(result["files"]))
        messagebox.showinfo("导出完成", "将 ZIP 安装为 MO2 中单独的覆盖模组，置于完整 FreeClimb 之后。\n\n" + result["output"])

    def run(self, message, operation, complete):
        if self.future is not None:
            return
        self.status.set(message)
        self.after_success = complete
        self.future = self.executor.submit(operation)
        for button in self.buttons:
            button.configure(state="disabled")
        self.slot_box.configure(state="disabled")

    def poll(self):
        if self.future is not None and self.future.done():
            future, complete = self.future, self.after_success
            self.future = None
            self.after_success = None
            for button in self.buttons:
                button.configure(state="normal")
            self.slot_box.configure(state="readonly")
            try:
                complete(future.result())
            except Exception as error:
                self.failure(error)
        self.window.after(100, self.poll)

    def set_details(self, text):
        self.details.configure(state="normal")
        self.details.delete("1.0", "end")
        self.details.insert("1.0", text)
        self.details.configure(state="disabled")

    def failure(self, error):
        self.status.set("操作失败；输入包和原有输出 ZIP 均未改动。")
        self.set_details(str(error))
        messagebox.showerror("无法完成操作", str(error))

    def close(self):
        if self.future is not None:
            messagebox.showinfo("仍在处理", "请等待当前读取或校验完成后再关闭。")
            return
        self.executor.shutdown(wait=False, cancel_futures=True)
        self.window.destroy()


def main():
    window = tk.Tk()
    AuthoringApp(window)
    window.mainloop()


if __name__ == "__main__":
    main()
