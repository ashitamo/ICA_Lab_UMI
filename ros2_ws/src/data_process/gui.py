import sys
import os
import json
import glob

from PyQt5.QtWidgets import (
    QApplication, QMainWindow, QWidget,
    QVBoxLayout, QHBoxLayout, QGridLayout,
    QLabel, QPushButton, QTextEdit,
    QMessageBox, QGroupBox,
    QListWidget, QListWidgetItem, QComboBox,
    QGraphicsView, QGraphicsScene, QGraphicsRectItem, QGraphicsSimpleTextItem,
    QLineEdit, QFileDialog
)
from PyQt5.QtCore import Qt, QRectF, QProcess
from PyQt5.QtGui import QMovie, QPen, QBrush


SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))

# === 預設資料夾（啟動時自動填入，可在 GUI 上修改）===
DEFAULT_DEMO_FOLDER = os.path.join(os.environ['HOME'], 'ICA_Lab_UMI/UMI_demo_24')
DEFAULT_EXE_FOLDER  = os.path.join(os.environ['HOME'], 'ICA_Lab_UMI/UMI_exe_24')
# ====================================================


class WorkflowView(QGraphicsView):
    """簡單工作流程圖：每個 atomic action 一個方塊 + 箭頭，並高亮選擇的 action。"""

    def __init__(self, parent=None):
        super().__init__(parent)
        self.scene = QGraphicsScene(self)
        self.setScene(self.scene)
        self.atomic_actions = []
        self.selected_index = -1

    def update_workflow(self, atomic_actions, selected_index):
        self.atomic_actions = atomic_actions
        self.selected_index = selected_index
        self._redraw()

    def _redraw(self):
        self.scene.clear()
        if not self.atomic_actions:
            return

        box_w, box_h = 120, 50
        spacing = 40
        y = 0

        for i, aa in enumerate(self.atomic_actions):
            x = i * (box_w + spacing)
            rect = QRectF(x, y, box_w, box_h)

            if i == self.selected_index:
                brush = QBrush(Qt.yellow)
                pen = QPen(Qt.red, 2)
            else:
                brush = QBrush(Qt.white)
                pen = QPen(Qt.black, 1)

            rect_item = QGraphicsRectItem(rect)
            rect_item.setBrush(brush)
            rect_item.setPen(pen)
            self.scene.addItem(rect_item)

            action_text = aa.get("action", "Unknown")
            text_item = QGraphicsSimpleTextItem(action_text)
            text_item.setPos(x + 5, y + 15)
            self.scene.addItem(text_item)

            # 畫箭頭（除了最後一個）
            if i < len(self.atomic_actions) - 1:
                arrow_x1 = x + box_w
                arrow_x2 = arrow_x1 + spacing - 10
                arrow_y = y + box_h / 2
                self.scene.addLine(arrow_x1, arrow_y, arrow_x2, arrow_y, QPen(Qt.black, 1))
                self.scene.addLine(arrow_x2 - 10, arrow_y - 5, arrow_x2, arrow_y, QPen(Qt.black, 1))
                self.scene.addLine(arrow_x2 - 10, arrow_y + 5, arrow_x2, arrow_y, QPen(Qt.black, 1))

        self.setSceneRect(self.scene.itemsBoundingRect())


class PipelineGUI(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Task Pipeline GUI (Qt)")
        self.resize(1100, 700)

        # QProcess for pipeline steps
        self.process = None
        self.command_queue = []

        # 動作編輯用資料
        self.task_data = None
        self.current_task_index = -1
        self.current_action_index = -1
        self.gif_movie = None

        self.init_ui()
        self.setFocusPolicy(Qt.StrongFocus)
    # ================= UI =================

    def init_ui(self):
        central = QWidget()
        self.setCentralWidget(central)
        main_layout = QVBoxLayout(central)

        # 1. 路徑設定（小區塊）
        self._init_path_area(main_layout)

        # 2. Pipeline + Modify 全部在同一個 Group 裡
        self._init_pipeline_and_editor(main_layout)

        # 3. 下方狀態 + LOG
        status_layout = QHBoxLayout()
        self.status_label = QLabel("就緒")
        status_layout.addWidget(QLabel("狀態："))
        status_layout.addWidget(self.status_label)
        status_layout.addStretch()
        main_layout.addLayout(status_layout)

        self.log_edit = QTextEdit()
        self.log_edit.setReadOnly(True)
        main_layout.addWidget(QLabel("輸出 Log："))
        main_layout.addWidget(self.log_edit, 1)

    # ---------- 共用：取得目前 GUI 上的路徑 ----------

    def get_paths(self):
        demo = self.demo_edit.text().strip()
        exe = self.exe_edit.text().strip()
        return demo, exe

    # ================= 路徑區 =================

    def _init_path_area(self, parent_layout):
        path_group = QGroupBox("路徑設定（預設值可修改）")
        grid = QGridLayout()
        path_group.setLayout(grid)

        self.demo_edit = QLineEdit(DEFAULT_DEMO_FOLDER)
        self.exe_edit  = QLineEdit(DEFAULT_EXE_FOLDER)

        btn_demo_browse = QPushButton("瀏覽…")
        btn_demo_browse.clicked.connect(self.browse_demo_folder)
        btn_exe_browse = QPushButton("瀏覽…")
        btn_exe_browse.clicked.connect(self.browse_exe_folder)

        grid.addWidget(QLabel("Demo (含 demo.pkl):"), 0, 0)
        grid.addWidget(self.demo_edit, 0, 1)
        grid.addWidget(btn_demo_browse, 0, 2)

        grid.addWidget(QLabel("Exe (task_segmentation / gifs):"), 1, 0)
        grid.addWidget(self.exe_edit, 1, 1)
        grid.addWidget(btn_exe_browse, 1, 2)

        parent_layout.addWidget(path_group)

    def browse_demo_folder(self):
        folder = QFileDialog.getExistingDirectory(self, "選擇 Demo 資料夾", self.demo_edit.text())
        if folder:
            self.demo_edit.setText(folder)

    def browse_exe_folder(self):
        folder = QFileDialog.getExistingDirectory(self, "選擇 Exe 資料夾", self.exe_edit.text())
        if folder:
            self.exe_edit.setText(folder)

    # ================= Pipeline + Modify 同頁 =================

    def _init_pipeline_and_editor(self, parent_layout):
        editor_group = QGroupBox("Pipeline + 動作編輯 (modify_action)")
        layout = QVBoxLayout(editor_group)

        # --- 上面一排：Pipeline 小按鈕 + Segmentation 載入 / 儲存 ---
        top_controls = QHBoxLayout()

        # Pipeline row（縮成一小排）
        pipe_row = QHBoxLayout()
        pipe_row.addWidget(QLabel("Pipeline:"))
        self.btn_step1 = QPushButton("1. Convert PKL")
        self.btn_step1.clicked.connect(self.run_step1_pkl_convert)
        self.btn_step2 = QPushButton("2. Task Segmentation")
        self.btn_step2.clicked.connect(self.run_step2_task_segmentation)
        self.btn_step4 = QPushButton("4. 更新 task_workflow (smooth)")
        self.btn_step4.clicked.connect(self.run_step4_update_workflow)
        self.btn_run_all = QPushButton("全部 (1→2→4)")
        self.btn_run_all.clicked.connect(self.run_all_steps)

        for b in (self.btn_step1, self.btn_step2, self.btn_step4, self.btn_run_all):
            pipe_row.addWidget(b)

        # Segmentation row（modify_action 專用）
        seg_row = QHBoxLayout()
        seg_row.addWidget(QLabel("Segmentation:"))
        self.btn_load_seg = QPushButton("載入 JSON")
        self.btn_load_seg.clicked.connect(self.load_segmentation_json)
        self.btn_save_seg = QPushButton("儲存 JSON 並更新 workflow")
        self.btn_save_seg.clicked.connect(self.save_segmentation_json)
        seg_row.addWidget(self.btn_load_seg)
        seg_row.addWidget(self.btn_save_seg)

        top_controls.addLayout(pipe_row)
        top_controls.addStretch()
        top_controls.addLayout(seg_row)

        layout.addLayout(top_controls)

        # --- 中間：左側 task/action，右側 GIF + 動作下拉 ---
        upper_layout = QHBoxLayout()

        # 左側：Primitive Task + Atomic Action 列表
        left_layout = QVBoxLayout()
        left_layout.addWidget(QLabel("Primitive Task："))

        self.task_combo = QComboBox()
        self.task_combo.currentIndexChanged.connect(self.on_task_changed)
        left_layout.addWidget(self.task_combo)

        left_layout.addWidget(QLabel("Atomic Actions："))
        self.action_list = QListWidget()
        self.action_list.currentRowChanged.connect(self.on_action_selected)
        left_layout.addWidget(self.action_list, 1)

        upper_layout.addLayout(left_layout, 2)

        # 右側：GIF + 動作編輯
        right_layout = QVBoxLayout()

        self.gif_label = QLabel("尚未載入 GIF")
        self.gif_label.setAlignment(Qt.AlignCenter)
        self.gif_label.setMinimumSize(320, 240)
        right_layout.addWidget(self.gif_label)

        form_layout = QGridLayout()
        form_layout.addWidget(QLabel("目前動作："), 0, 0)
        self.action_combo = QComboBox()
        self.base_actions = ["Unknown", "Approach", "Grasp", "Move", "Insert", "Retreat"]
        self.action_combo.addItems(self.base_actions)
        form_layout.addWidget(self.action_combo, 0, 1)

        self.btn_apply_action = QPushButton("套用動作到該片段")
        self.btn_apply_action.clicked.connect(self.apply_current_action)
        form_layout.addWidget(self.btn_apply_action, 1, 0, 1, 2)

        right_layout.addLayout(form_layout)
        right_layout.addStretch()

        upper_layout.addLayout(right_layout, 3)

        layout.addLayout(upper_layout, 2)

        # --- 下方：工作流程圖 ---
        layout.addWidget(QLabel("工作流程圖 (Atomic Actions 流程)："))
        self.workflow_view = WorkflowView()
        self.workflow_view.setMinimumHeight(180)
        layout.addWidget(self.workflow_view, 1)

        parent_layout.addWidget(editor_group)

    # =============== Pipeline：路徑檢查 ===============

    def ensure_paths(self):
        demo_folder, exe_folder = self.get_paths()

        if not demo_folder:
            QMessageBox.critical(self, "錯誤", "請先設定 Demo 資料夾路徑")
            return False
        if not exe_folder:
            QMessageBox.critical(self, "錯誤", "請先設定 Exe 資料夾路徑")
            return False

        if not os.path.isdir(demo_folder):
            QMessageBox.critical(self, "錯誤", f"Demo 資料夾不存在：\n{demo_folder}")
            return False

        os.makedirs(exe_folder, exist_ok=True)
        return True

    # =============== Pipeline 按鈕 ===============

    def run_step1_pkl_convert(self):
        if not self.ensure_paths():
            return
        demo_folder, exe_folder = self.get_paths()

        input_pkl = os.path.join(demo_folder, "demo.pkl")
        if not os.path.isfile(input_pkl):
            QMessageBox.critical(self, "錯誤", f"找不到 demo.pkl：\n{input_pkl}")
            return

        script_path = os.path.join(SCRIPT_DIR, "pkl_convert.py")
        args = [script_path, input_pkl]

        self.start_process("Convert PKL", args)

    def run_step2_task_segmentation(self):
        if not self.ensure_paths():
            return
        demo_folder, exe_folder = self.get_paths()

        script_path = os.path.join(SCRIPT_DIR, "task_segmentation.py")
        args = [script_path, demo_folder, exe_folder]
        self.command_queue = []
        self.start_process("Task Segmentation", args)

    def run_step4_update_workflow(self):
        """Step4：使用 smooth 版本更新 task_workflow.json"""
        if not self.ensure_paths():
            return
        demo_folder, exe_folder = self.get_paths()
        self.run_merge_and_smooth(demo_folder, exe_folder)

    def run_all_steps(self):
        """全部：1 → 2 → (merge_action + smooth)"""
        if not self.ensure_paths():
            return
        demo_folder, exe_folder = self.get_paths()

        merge_script  = os.path.join(SCRIPT_DIR, "merge_action.py")
        smooth_script = os.path.join(SCRIPT_DIR, "task_manager_traject_smooth.py")

        missing = []
        if not os.path.isfile(merge_script):
            missing.append(merge_script)
        if not os.path.isfile(smooth_script):
            missing.append(smooth_script)

        if missing:
            QMessageBox.warning(
                self, "找不到腳本",
                "以下腳本不存在，無法完成 4：\n" + "\n".join(missing)
            )
            return

        self.command_queue = []

        script1 = os.path.join(SCRIPT_DIR, "pkl_convert.py")
        script2 = os.path.join(SCRIPT_DIR, "task_segmentation.py")

        self.command_queue.append(("Convert PKL", [script1, os.path.join(demo_folder, "demo.pkl")]))
        self.command_queue.append(("Task Segmentation", [script2, demo_folder, exe_folder]))
        self.command_queue.append(("merge_action", [merge_script, exe_folder]))
        self.command_queue.append(("task_manager_traject_smooth", [smooth_script, demo_folder, exe_folder]))

        self.log_edit.append("=== 開始全部執行 (1 → 2 → merge → smooth) ===")
        self.start_next_from_queue()

    # =============== QProcess 控制 ===============

    def start_next_from_queue(self):
        if not self.command_queue:
            self.status_label.setText("全部步驟執行完成")
            self.log_edit.append("=== 全部步驟完成 ===\n")
            return

        name, args = self.command_queue.pop(0)
        self.start_process(name, args)

    def start_process(self, name, script_and_args):
        if self.process is not None and self.process.state() != QProcess.NotRunning:
            QMessageBox.warning(self, "執行中", "目前有程式在執行中，請先等它跑完。")
            return

        self.status_label.setText(f"執行中：{name}")
        python_exe = sys.executable

        self.process = QProcess(self)

        # 關鍵1：讓 stdout / stderr 合併，一起從 readAll() 拿
        self.process.setProcessChannelMode(QProcess.MergedChannels)

        # 關鍵2：用 -u 啟動 python，關閉 buffer，print 會即時輸出
        args = ['-u'] + script_and_args
        self.process.setProgram(python_exe)
        self.process.setArguments(args)
        self.process.setWorkingDirectory(SCRIPT_DIR)

        # 只需要一個 readyRead 事件就好（因為已經 merged）
        self.process.readyReadStandardOutput.connect(self.handle_ready_read)
        self.process.finished.connect(self.handle_finished)

        cmd_str = " ".join([python_exe] + args)
        self.log_edit.append(f"\n$ {cmd_str}\n")

        self.process.start()


    def handle_ready_read(self):
        if not self.process:
            return
        # 因為 channel 已經 merged，直接 readAll()
        data = self.process.readAll()
        text = bytes(data).decode("utf-8", errors="ignore")
        self.log_edit.moveCursor(self.log_edit.textCursor().End)
        self.log_edit.insertPlainText(text)
        self.log_edit.moveCursor(self.log_edit.textCursor().End)


    def handle_finished(self, exit_code, exit_status):
        if exit_code == 0:
            self.log_edit.append("\n>>> 執行完成（exit code 0）\n")
        else:
            self.log_edit.append(f"\n>>> 執行結束，exit code = {exit_code}\n")

        if self.command_queue:
            self.start_next_from_queue()
        else:
            self.status_label.setText("就緒")

        self.process = None

    # =============== 動作編輯器：載入 / 儲存 / 更新 workflow ===============

    def load_segmentation_json(self):
        demo_folder, exe_folder = self.get_paths()
        if not exe_folder:
            QMessageBox.critical(self, "錯誤", "請先設定 Exe 資料夾路徑")
            return

        if not os.path.isdir(exe_folder):
            QMessageBox.critical(self, "錯誤", f"Exe 資料夾不存在：\n{exe_folder}")
            return

        json_path = os.path.join(exe_folder, "task_segmentation.json")
        if not os.path.isfile(json_path):
            QMessageBox.critical(self, "錯誤", f"找不到 task_segmentation.json：\n{json_path}")
            return

        try:
            with open(json_path, "r", encoding="utf-8") as f:
                self.task_data = json.load(f)
        except Exception as e:
            QMessageBox.critical(self, "錯誤", f"讀取 JSON 失敗：\n{e}")
            return

        self.task_combo.clear()
        self.action_list.clear()
        self.current_task_index = -1
        self.current_action_index = -1

        primitive_tasks = self.task_data.get("primitive_tasks", [])
        for pt in primitive_tasks:
            tid = pt.get("task_id", "?")
            name = pt.get("primitive_tasks_name", "Unnamed")
            self.task_combo.addItem(f"{tid}: {name}", tid)

        if primitive_tasks:
            self.task_combo.setCurrentIndex(0)

        QMessageBox.information(self, "完成", "已載入 task_segmentation.json")

    def save_segmentation_json(self):
        if self.task_data is None:
            QMessageBox.warning(self, "提示", "尚未載入 task_segmentation.json")
            return

        demo_folder, exe_folder = self.get_paths()
        if not exe_folder:
            QMessageBox.critical(self, "錯誤", "請先設定 Exe 資料夾路徑")
            return

        json_path = os.path.join(exe_folder, "task_segmentation.json")
        try:
            with open(json_path, "w", encoding="utf-8") as f:
                json.dump(self.task_data, f, indent=2, ensure_ascii=False)
        except Exception as e:
            QMessageBox.critical(self, "錯誤", f"寫入 JSON 失敗：\n{e}")
            return

        QMessageBox.information(self, "完成", f"已儲存到：\n{json_path}")

        # 儲存後一樣跑 merge_action + smooth
        self.run_merge_and_smooth(demo_folder, exe_folder)

    def run_merge_and_smooth(self, demo_folder, exe_folder):
        merge_script  = os.path.join(SCRIPT_DIR, "merge_action.py")
        smooth_script = os.path.join(SCRIPT_DIR, "task_manager_traject_smooth.py")

        missing = []
        if not os.path.isfile(merge_script):
            missing.append(merge_script)
        if not os.path.isfile(smooth_script):
            missing.append(smooth_script)

        if missing:
            QMessageBox.warning(
                self, "找不到腳本",
                "以下腳本不存在，無法更新 workflow：\n" + "\n".join(missing)
            )
            return

        if self.process is not None and self.process.state() != QProcess.NotRunning:
            QMessageBox.warning(
                self, "執行中",
                "目前有其它流程在執行中，暫時無法執行 merge_action / task_manager_traject_smooth。"
            )
            return

        self.command_queue = []
        self.command_queue.append(("merge_action", [merge_script, exe_folder]))
        self.command_queue.append(("task_manager_traject_smooth", [smooth_script, demo_folder, exe_folder]))

        self.log_edit.append("=== 執行：merge_action.py & task_manager_traject_smooth.py ===")
        self.start_next_from_queue()

    # =============== 動作編輯：列表 / GIF / 修改 action ===============

    def on_task_changed(self, index):
        if self.task_data is None:
            return

        primitive_tasks = self.task_data.get("primitive_tasks", [])
        if index < 0 or index >= len(primitive_tasks):
            return

        self.current_task_index = index
        self.action_list.clear()
        self.current_action_index = -1

        pt = primitive_tasks[index]
        atomic_actions = pt.get("atomic_actions", [])
        for i, aa in enumerate(atomic_actions):
            action_name = aa.get("action", "Unknown")
            conf = aa.get("confidence", "")
            item_text = f"{i:02d}: {action_name} ({conf})"
            self.action_list.addItem(QListWidgetItem(item_text))

        self.workflow_view.update_workflow(atomic_actions, -1)
        self.clear_gif()

    def on_action_selected(self, row):
        if self.task_data is None:
            return
        if self.current_task_index < 0:
            return

        primitive_tasks = self.task_data.get("primitive_tasks", [])
        if self.current_task_index >= len(primitive_tasks):
            return
        pt = primitive_tasks[self.current_task_index]
        atomic_actions = pt.get("atomic_actions", [])

        if row < 0 or row >= len(atomic_actions):
            self.current_action_index = -1
            self.workflow_view.update_workflow(atomic_actions, -1)
            self.clear_gif()
            return

        self.current_action_index = row
        aa = atomic_actions[row]

        current_action = aa.get("action", "Unknown")
        if current_action not in self.base_actions:
            if self.action_combo.findText(current_action) == -1:
                self.action_combo.addItem(current_action)
        idx = self.action_combo.findText(current_action)
        if idx >= 0:
            self.action_combo.setCurrentIndex(idx)

        self.workflow_view.update_workflow(atomic_actions, row)
        self.update_gif_view()

    def clear_gif(self):
        if self.gif_movie is not None:
            self.gif_movie.stop()
            self.gif_movie = None
        self.gif_label.setText("尚未載入 GIF")

    def update_gif_view(self):
        self.clear_gif()
        if self.task_data is None:
            return
        if self.current_task_index < 0 or self.current_action_index < 0:
            return

        primitive_tasks = self.task_data.get("primitive_tasks", [])
        if self.current_task_index >= len(primitive_tasks):
            return
        pt = primitive_tasks[self.current_task_index]
        task_id = pt.get("task_id")

        demo_folder, exe_folder = self.get_paths()
        gif_root = os.path.join(exe_folder, "atomic_action_gifs")
        task_gif_dir = os.path.join(gif_root, f"task_{task_id}")

        if not os.path.isdir(task_gif_dir):
            self.gif_label.setText(f"找不到 GIF 資料夾：\n{task_gif_dir}")
            return

        seg_idx = self.current_action_index
        pattern = os.path.join(task_gif_dir, f"segment_{seg_idx:02d}_*.gif")
        matches = glob.glob(pattern)

        if not matches:
            self.gif_label.setText(f"找不到對應 GIF：\nsegment_{seg_idx:02d}_*.gif")
            return

        gif_path = matches[0]
        self.gif_movie = QMovie(gif_path)
        self.gif_label.setMovie(self.gif_movie)
        self.gif_movie.start()

    def apply_current_action(self, show_message=True):
        if self.task_data is None:
            if show_message:
                QMessageBox.warning(self, "提示", "尚未載入 task_segmentation.json")
            return
        if self.current_task_index < 0 or self.current_action_index < 0:
            if show_message:
                QMessageBox.warning(self, "提示", "請先選擇一個 atomic action")
            return

        new_action = self.action_combo.currentText().strip()
        if not new_action:
            if show_message:
                QMessageBox.warning(self, "提示", "動作名稱不可為空")
            return

        primitive_tasks = self.task_data.get("primitive_tasks", [])
        pt = primitive_tasks[self.current_task_index]
        atomic_actions = pt.get("atomic_actions", [])

        aa = atomic_actions[self.current_action_index]
        aa["action"] = new_action  # 寫回 JSON 結構

        # 更新左邊列表文字
        conf = aa.get("confidence", "")
        item_text = f"{self.current_action_index:02d}: {new_action} ({conf})"
        self.action_list.item(self.current_action_index).setText(item_text)

        # 更新流程圖高亮
        self.workflow_view.update_workflow(atomic_actions, self.current_action_index)

        if show_message:
            QMessageBox.information(self, "完成", f"已更新該片段的動作為：{new_action}")


    def keyPressEvent(self, event):
        """用左右方向鍵直接更改目前片段的 action。"""
        key = event.key()

        if key in (Qt.Key_Left, Qt.Key_Right):
            # 沒有選任何片段就不處理
            if (
                self.task_data is None
                or self.current_task_index < 0
                or self.current_action_index < 0
            ):
                return

            count = self.action_combo.count()
            if count == 0:
                return

            cur_idx = self.action_combo.currentIndex()
            if cur_idx < 0:
                cur_idx = 0

            if key == Qt.Key_Right:
                new_idx = (cur_idx + 1) % count
            else:  # Qt.Key_Left
                new_idx = (cur_idx - 1 + count) % count

            # 換 combobox 選項
            self.action_combo.setCurrentIndex(new_idx)
            # 直接套用到目前片段，但不要跳出訊息框
            self.apply_current_action(show_message=False)
        else:
            # 其他按鍵仍交給原本的處理（例如上下切換 list）
            super().keyPressEvent(event)




def main():
    app = QApplication(sys.argv)
    win = PipelineGUI()
    win.show()
    sys.exit(app.exec_())


if __name__ == "__main__":
    main()


'''
=== 開始全部執行 (1 → 2 → merge → smooth) ===

$ /bin/python3 /home/lab606/ICA_Lab_UMI/ros2_ws/src/data_process/pkl_convert.py /home/lab606/ICA_Lab_UMI/UMI_demo_24/data_list.pkl
Attempting to load data from '/home/lab606/ICA_Lab_UMI/UMI_demo_24/data_list.pkl'...
Error: The input file '/home/lab606/ICA_Lab_UMI/UMI_demo_24/data_list.pkl' was not found.
Data will be saved in '/home/lab606/ICA_Lab_UMI/UMI_demo_24' directory.
Traceback (most recent call last):
  File "/home/lab606/ICA_Lab_UMI/ros2_ws/src/data_process/pkl_convert.py", line 98, in <module>
    convert_pkl_file(input_path=args.input, output_folder=input_dir)
  File "/home/lab606/ICA_Lab_UMI/ros2_ws/src/data_process/pkl_convert.py", line 43, in convert_pkl_file
    demo_start_time = data_from_pickle[0][0]
UnboundLocalError: local variable 'data_from_pickle' referenced before assignment


>>> 執行結束，exit code = 1


$ /bin/python3 /home/lab606/ICA_Lab_UMI/ros2_ws/src/data_process/task_segmentation.py /home/lab606/ICA_Lab_UMI/UMI_demo_24 /home/lab606/ICA_Lab_UMI/UMI_exe_24

'''