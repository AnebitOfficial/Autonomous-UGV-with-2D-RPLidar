import sys
from PyQt5.QtWidgets import QApplication, QMainWindow, QStackedWidget, QWidget, QVBoxLayout, QPushButton, QLabel, QComboBox,QListWidget,QLineEdit, QDialog,QGraphicsView,QGraphicsScene,QGraphicsEllipseItem,QGraphicsTextItem,QMenu,QGraphicsItem,QSplitter
from PyQt5.QtCore import Qt, QRectF, QPointF
from PyQt5.QtGui import QPixmap,QPainter, QBrush, QColor, QFont, QPen, QKeyEvent
import time
import subprocess
import qdarkstyle


def run_in_new_terminal(command,wait_or_not=True):
    s = 'nohup ' + command  +  '> nohup.txt &'
    process = subprocess.Popen(s, shell=True,stdin=subprocess.PIPE)
    if wait_or_not:
        process.wait()
    
def kill_terminal(command):
    try:
        c = 'ps aux | grep ' + "\'" + command + "\'"
        result = subprocess.run(c, shell=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
        output = result.stdout
        if len(output.split()) < 1:
            return
        pid = output.split()[1]
        s = "kill -9 " +str(pid) 
        print(s)
        subprocess.run(s, shell=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
        return pid
    except subprocess.CalledProcessError:
        print(command,': No terminal processes found')
        return None

class HelpDialog(QDialog):
    def __init__(self, help_text):
        super().__init__()
        self.setWindowTitle("Help Information")
        self.setGeometry(100, 100, 400, 200)
        layout = QVBoxLayout()
        label = QLabel(help_text)
        layout.addWidget(label)
        self.setLayout(layout)
        
class InteractivePoint(QGraphicsEllipseItem):
    def __init__(self, x, y):
        super().__init__(QRectF(x - 10, y - 10, 20, 20))
        self.setFlags(QGraphicsItem.ItemIsSelectable)
        self.setBrush(QColor("white"))
        self.label = None

class CrossGraphic(QGraphicsItem):
    def __init__(self, center, size=30, parent=None):
        super().__init__(parent)
        self.center = center
        self.size = size
        self.label = QGraphicsTextItem(f"({self.center.x():.2f}, {self.center.y():.2f})", self)
        self.label.setDefaultTextColor(QColor("blue"))
        self.label.setFont(QFont("Arial", 50))
        self.label.setPos(self.center.x() + 10, self.center.y() - 20)

    def boundingRect(self):
        return QRectF(self.center.x() - self.size, self.center.y() - self.size,
                      self.size * 2, self.size * 2 + 20)

    def paint(self, painter, option, widget):
        painter.setPen(QPen(QColor("red"), 10, Qt.SolidLine))
        painter.drawLine(QPointF(self.center.x(), self.center.y() - self.size),
                         QPointF(self.center.x(), self.center.y() + self.size))
        painter.drawLine(QPointF(self.center.x() - self.size, self.center.y()),
                         QPointF(self.center.x() + self.size, self.center.y()))

class CustomGraphicsView(QGraphicsView):
    def __init__(self, scene, parent=None):
        super().__init__(scene, parent)
        self.setRenderHints(QPainter.Antialiasing)
        self.setDragMode(QGraphicsView.RubberBandDrag)
        self.pending_label = None

    def wheelEvent(self, event):
        zoomInFactor = 1.25
        zoomOutFactor = 1 / zoomInFactor
        self.setTransformationAnchor(QGraphicsView.AnchorUnderMouse)
        if event.angleDelta().y() > 0:
            self.scale(zoomInFactor, zoomInFactor)
        else:
            self.scale(zoomOutFactor, zoomOutFactor)

    def keyPressEvent(self, event):
        if event.key() == Qt.Key_Space:
            self.write_crosses_to_file('crosses_coordinates.txt')
        else:
            super().keyPressEvent(event)

    def write_crosses_to_file(self, filename):
        with open(filename, 'w') as file:
            for item in self.scene().items():
                if isinstance(item, CrossGraphic):
                    file.write(f"{item.center.x():.2f}, {item.center.y():.2f}\n")

    def mousePressEvent(self, event):
        if self.pending_label:
            scenePos = self.mapToScene(event.pos())
            self.pending_label.setPos(scenePos)
            self.pending_label = None
        elif event.modifiers() & Qt.ControlModifier:
            if event.button() == Qt.RightButton:  # Remove crosses
                scenePos = self.mapToScene(event.pos())
                items = self.scene().items(scenePos)
                for item in items:
                    if isinstance(item, CrossGraphic):
                        self.scene().removeItem(item)
            else:  # Add crosses
                scenePos = self.mapToScene(event.pos())
                cross = CrossGraphic(scenePos)
                self.scene().addItem(cross)
        else:
            super().mousePressEvent(event)

class App(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle('Anebit')
        self.setGeometry(300, 300, 1100, 700)
        self.stacked_widget = QStackedWidget()
        self.setCentralWidget(self.stacked_widget)
        self.init_ui()

    def init_ui(self):
        main_page = MainPage(self.change_page)
        subpage1 = SubPage1(self.change_page)
        subpage2 = SubPage2(self.change_page)       
        subpage3 = SubPage3(self.change_page)

        # Add pages to the stack
        self.stacked_widget.addWidget(main_page)  # index 0
        self.stacked_widget.addWidget(subpage1)   # index 1
        self.stacked_widget.addWidget(subpage2)   # index 2       
        self.stacked_widget.addWidget(subpage3)   #index 3

    def change_page(self, index,additional_work=False):
        self.stacked_widget.setCurrentIndex(index)
        if additional_work:
            kill_terminal("roscore")
            kill_terminal("move_base_controller.py")
            kill_terminal('motorConrolWASD.py')
            kill_terminal("giveMoveBaseGoal.py")
            kill_terminal("ard_v3.py")

    def closeEvent(self, event):
        kill_terminal("roscore")
        kill_terminal("move_base_controller.py")
        kill_terminal('motorConrolWASD.py')
        kill_terminal("ard_v3.py")
        kill_terminal("giveMoveBaseGoal.py")
        super().closeEvent(event)

class MainPage(QWidget):
    def __init__(self, change_page_fn):
        super().__init__()
        layout = QVBoxLayout()

        # Load and display an image
        image = QLabel()
        pixmap = QPixmap('anebit.png')  # Replace with your image path
        image.setPixmap(pixmap)
        image.setAlignment(Qt.AlignCenter)
        layout.addWidget(image)
        
        self.help_btn = QPushButton('Help')
        self.help_btn.clicked.connect(self.show_help)
        layout.addWidget(self.help_btn)

        btn_to_subpage1 = QPushButton('Navigate')
        btn_to_subpage1.clicked.connect(lambda: change_page_fn(1))
        layout.addWidget(btn_to_subpage1)

        btn_to_subpage2 = QPushButton('Create a New 3D Map')
        btn_to_subpage2.clicked.connect(self.on_create_new_map)
        layout.addWidget(btn_to_subpage2)
              
        btn_to_subpage3 = QPushButton('Create a 2D Map via Hector-SLAM')
        btn_to_subpage3.clicked.connect(lambda: change_page_fn(3))
        layout.addWidget(btn_to_subpage3)

        self.setLayout(layout)
        self.change_page_fn = change_page_fn
        

        
    def on_create_new_map(self):
        run_in_new_terminal("python3 maps/clear.py")
        self.change_page_fn(2)
        
    def show_help(self):
            help_text = "This is the main page. Use this page to navigate to other parts of the application. \n You can use 'Create a New 3D Map' button to go to 3D mapping page. \n Once you selected this option, it will delete the previos maps.\n You can use 'Create a 2D Map via Hector-SLAM' button to create a 2D map of your environment via Hector-SLAM algorithm.\n You can use 'Navigate' button to navigate around the most recent map that you recorded with Hector SLAM."
            dlg = HelpDialog(help_text)
            dlg.exec_()


class SubPage1(QWidget):
    def __init__(self, change_page_fn):
        super().__init__()
        self.initUI(change_page_fn)
        self.labels_dict = {}

    def initUI(self,change_page_fn):
        self.layout = QVBoxLayout(self)

        self.splitter = QSplitter(Qt.Horizontal)
        self.layout.addWidget(self.splitter)

        self.scene = QGraphicsScene()
        self.scene.setSceneRect(0, 0, 800, 600)
        self.view = CustomGraphicsView(self.scene, self)
        self.splitter.addWidget(self.view)

        self.side_panel = QWidget()
        self.side_layout = QVBoxLayout(self.side_panel)
        self.labels_view = QListWidget()
        self.side_layout.addWidget(self.labels_view)

        self.label_input = QLineEdit()
        self.label_input.setPlaceholderText("Enter label name")
        self.side_layout.addWidget(self.label_input)

        self.label_button = QPushButton("Label Selected")
        self.label_button.clicked.connect(self.label_selected)
        self.side_layout.addWidget(self.label_button)

        self.set_label_button = QPushButton("Set Label")
        self.set_label_button.setEnabled(False)
        self.set_label_button.clicked.connect(self.set_label)
        self.side_layout.addWidget(self.set_label_button)

        self.refresh_label_button = QPushButton("Update Label Position")
        self.refresh_label_button.setEnabled(False)
        self.refresh_label_button.clicked.connect(self.refresh_label_position)
        self.side_layout.addWidget(self.refresh_label_button)

        self.go_to_label_button = QPushButton("Go to Label")
        self.go_to_label_button.setEnabled(False)
        self.go_to_label_button.clicked.connect(self.go_to_label)
        self.side_layout.addWidget(self.go_to_label_button)

        self.remove_button = QPushButton("Remove Selected Labels")
        self.remove_button.setEnabled(False)
        self.remove_button.clicked.connect(self.remove_selected_labels)
        self.side_layout.addWidget(self.remove_button)
        
        self.start_btn = QPushButton("Start Autonomous Navigation")
        self.start_btn.clicked.connect(self.start)
        self.side_layout.addWidget(self.start_btn)
        
        self.stop_btn = QPushButton("Stop Autonomous Navigation")
        self.stop_btn.setEnabled(False)
        self.stop_btn.clicked.connect(self.stop)
        self.side_layout.addWidget(self.stop_btn)
        
        self.start_manual_btn = QPushButton("Start Manual Navigation")
        self.start_manual_btn.clicked.connect(self.start_manual)
        self.side_layout.addWidget(self.start_manual_btn)
        
        
        self.back_btn = QPushButton('Return to Main Page')
        self.back_btn.clicked.connect(lambda: change_page_fn(0,True))
        self.side_layout.addWidget(self.back_btn)
        
        self.splitter.addWidget(self.side_panel)

        points = self.read_points_from_file('maps/hector_map.txt')
        self.load_and_display_points(points)

        #self.view.setContextMenuPolicy(Qt.CustomContextMenu)
        #self.view.customContextMenuRequested.connect(self.context_menu)
        self.setGeometry(300, 300, 1100, 700)
        self.setWindowTitle('Navigate')
        
    def showEvent(self, event):
        print("2D map loaded")
        points = self.read_points_from_file('maps/hector_map.txt')
        self.load_and_display_points(points)
        super().showEvent(event)
        
    def start(self):
        run_in_new_terminal("python3 move_base_controller.py")
        self.stop_btn.setEnabled(True)
        self.start_btn.setEnabled(False)
    
    def stop(self):
        kill_terminal("roscore")
        kill_terminal("locnew2.launch")
        kill_terminal('move_base.launch')
        kill_terminal("move_base_controller.py")
        kill_terminal("giveMoveBaseGoal.py")
        self.start_btn.setEnabled(True)
        self.stop_btn.setEnabled(False)
    
    def start_manual(self):
        run_in_new_terminal("python3 maps/motorControlWASD.py N")

    def label_selected(self):
        label = self.label_input.text().strip()
        if label:
            self.label_selected_items(label, QColor("green"))
            self.set_label_button.setEnabled(True)
            self.refresh_label_button.setEnabled(True)
            self.go_to_label_button.setEnabled(True)
            self.remove_button.setEnabled(True)

    def set_label(self):
        current_item = self.labels_view.currentItem()
        if current_item:
            label_name = current_item.text()
            label_data = self.labels_dict.get(label_name)
            if label_data:
                self.view.pending_label = label_data[0]
                pos = self.view.pending_label.pos()
                self.update_label_coordinates(label_name, pos)


    def refresh_label_position(self):
        current_item = self.labels_view.currentItem()
        if current_item:
            label_name = current_item.text()
            label, _ = self.labels_dict.get(label_name, (None, None))
            if label:
                current_pos = label.pos()
                self.update_label_coordinates(label_name, current_pos)

    def go_to_label(self):
        current_item = self.labels_view.currentItem()
        if current_item:
            label_name = current_item.text()
            label, _ = self.labels_dict.get(label_name, (None, None))            
            if label:
                x = -label.pos().x()/1000
                y = label.pos().y()/1000
                x = str(x)
                y = str(y)
                command = "python3 giveMoveBaseGoal.py " + x + " " + y
                print(command)
                run_in_new_terminal(command)

    def remove_selected_labels(self):
        selected_labels = [item.text() for item in self.labels_view.selectedItems()]
        for label in selected_labels:
            label_data = self.labels_dict.get(label)
            if label_data:
                self.scene.removeItem(label_data[0])  # Remove the label from the scene
                for point in label_data[1]:
                    point.setBrush(QBrush(QColor("white")))  # Reset color
                row = self.labels_view.row(self.labels_view.findItems(label, Qt.MatchExactly)[0])
                self.labels_view.takeItem(row)


    def read_points_from_file(self, filepath):
        points = []
        try:
            with open(filepath, 'r') as file:
                data = file.read().strip()
                points_data = data.split('),(')
                for point in points_data:
                    xy = point.replace('(', '').replace(')', '').split(',')
                    x, y = -float(xy[1]), -float(xy[0])
                    points.append((x, y))
        except Exception as e:
            print(f"Failed to read or parse the file: {e}")
        return points

    def load_and_display_points(self, points):
        if not points:
            return
        self.scene.clear()
        min_x = min_y = float('inf')
        max_x = max_y = float('-inf')
        for x, y in points:
            point = InteractivePoint(x, y)
            self.scene.addItem(point)
            min_x, max_x = min(min_x, x), max(max_x, x)
            min_y, max_y = min(min_y, y), max(max_y, y)

        margin = 10000  # Adjust margin size as needed
        rect = QRectF(min_x - margin, min_y - margin, max_x - min_x + 2 * margin, max_y - min_y + 2 * margin)
        self.scene.setSceneRect(rect)
    '''
    def context_menu(self, position):
        context_menu = QMenu(self.view)
        chair_action = context_menu.addAction("Label as Chair")
        table_action = context_menu.addAction("Label as Table")
        custom_action = context_menu.addAction("Custom Label")
        action = context_menu.exec_(self.view.mapToGlobal(position))
        if action:
            if action == custom_action:
                self.label_input.setFocus()
            else:
                label = "Chair" if action == chair_action else "Table"
                color = QColor("red") if action == chair_action else QColor("blue")
                self.label_selected_items(label, color)
                self.set_label_button.setEnabled(True)
                self.refresh_label_button.setEnabled(True)
                self.go_to_label_button.setEnabled(True)
                self.remove_button.setEnabled(True)
    '''
    def label_selected_items(self, label, color):
        selected_items = self.view.scene().selectedItems()
        if not selected_items:
            return
        mean_x, mean_y = 0, 0
        label_items = []
        for item in selected_items:
            if isinstance(item, InteractivePoint):
                item.setBrush(QBrush(color))
                mean_x += item.rect().center().x()
                mean_y += item.rect().center().y()
                label_items.append(item)

        if label_items:
            mean_x /= len(selected_items)
            mean_y /= len(selected_items)

            group_label = QGraphicsTextItem(f"{label}\n({mean_x:.2f}, {mean_y:.2f})")
            group_label.setFont(QFont("Arial", 72))
            group_label.setDefaultTextColor(Qt.white)
            group_label.setPos(mean_x, mean_y)
            self.scene.addItem(group_label)
            self.labels_dict[label] = (group_label, label_items)

            if label not in [self.labels_view.item(i).text() for i in range(self.labels_view.count())]:
                self.labels_view.addItem(label)
            print(f"Label '{label}' added at position ({mean_x}, {mean_y}) with {len(label_items)} items.")
        else:
            print("No InteractivePoint items were found among selected items.")

    def remove_selected_labels(self):
        selected_labels = [item.text() for item in self.labels_view.selectedItems()]
        for label in selected_labels:
            label_data = self.labels_dict.get(label)
            if label_data:
                self.scene.removeItem(label_data[0])  # Remove the label from the scene
                for point in label_data[1]:
                    point.setBrush(QBrush(QColor("white")))  # Reset color
            self.labels_view.takeItem(self.labels_view.row(self.labels_view.findItems(label, Qt.MatchExactly)[0]))

    def update_label_coordinates(self, label_name, pos):
        label, _ = self.labels_dict.get(label_name)
        if label:
            label.setPlainText(f"{label_name}\n({pos.x():.2f}, {pos.y():.2f})")

class SubPage2(QWidget):
    def __init__(self, change_page_fn):
        super().__init__()
        layout = QVBoxLayout()

        title = QLabel('Create a New 3D Map')
        title.setAlignment(Qt.AlignCenter)
        layout.addWidget(title)
        
        self.help_btn = QPushButton('Help')
        self.help_btn.clicked.connect(self.show_help)
        layout.addWidget(self.help_btn)
        
        # Combo box for selecting options
        self.combo_navigate_options = QComboBox()
        self.combo_navigate_options.addItems(["Use SLAM", "Don't Use SLAM"])  # Add your specific options here

        self.btn_navigate = QPushButton('Navigate to New Point')
        self.btn_acquire = QPushButton('Acquire Data')
        self.btn_merge = QPushButton('Finalize Map')

        # Connecting buttons to the unified callback
        self.btn_navigate.clicked.connect(lambda: self.handle_button_click('navigate'))
        self.btn_acquire.clicked.connect(lambda: self.handle_button_click('acquire'))

        nav_layout = QVBoxLayout()
        nav_layout.addWidget(self.combo_navigate_options)
        nav_layout.addWidget(self.btn_navigate)
        layout.addLayout(nav_layout)
        layout.addWidget(self.btn_acquire)

        back_btn = QPushButton('Return to Main Page')
        back_btn.clicked.connect(lambda: change_page_fn(0,True))
        layout.addWidget(back_btn)

        self.setLayout(layout)
        
    def show_help(self):
        help_text = "This is the 3D map construction page. You can navigate around the room by clicking 'Navigate to New Point' button. \n Depending on your choice, it will start Hector-SLAM algorithm and you can use 'WASD' keys of your keyboard to move the Oedipus forward-left-backward-right. \n You can use 'X' to stop the motors and 'ESC' key to stop navigation. Wait until SLAM algorithm to finalize. \n 'Don't Use SLAM' option could be used to adjust the initial position of the robot, but when gathering data 'Use SLAM' option should be used. \n If 'Use SLAM' option is selected, a RVIZ page will be open to show the status and location of the robot.\n You can use 'Acquire Data' button to get data with lidar. Wait until lidar scan the whole angle range and returns back to its initial position. \n You can return back to the main page by clicking 'Return to Main Page' button."
        dlg = HelpDialog(help_text)
        dlg.exec_()
            
    def handle_button_click(self, action):
        # Disable all buttons
        self.btn_navigate.setDisabled(True)
        self.btn_acquire.setDisabled(True)

        # Simulate processing
        if action == 'navigate':
            self.navigate_to_new_point()
        elif action == 'acquire':
            self.acquire_data()

        # Re-enable all buttons
        self.btn_navigate.setEnabled(True)
        self.btn_acquire.setEnabled(True)


    def navigate_to_new_point(self):
        # Simulate action
        selected_option = self.combo_navigate_options.currentText()

        if selected_option == "Use SLAM":
            run_in_new_terminal("python3 maps/motorControlWASD.py Y")
        elif selected_option == "Don't Use SLAM":
            run_in_new_terminal("python3 maps/motorControlWASD.py N")

    def acquire_data(self):
        # Simulate action
        run_in_new_terminal("python3 maps/ard_v3.py")

        
class SubPage3(QWidget):
    def __init__(self, change_page_fn):
        super().__init__()
        layout = QVBoxLayout()
        title = QLabel('Create a New 2D Map via Hector-SLAM')
        title.setAlignment(Qt.AlignCenter)
        layout.addWidget(title) 
        
        self.help_btn = QPushButton('Help')
        self.help_btn.clicked.connect(self.show_help)
        layout.addWidget(self.help_btn)  
            
        # Combo box for selecting options
        self.combo_navigate_options = QComboBox()
        self.combo_navigate_options.addItems(["Save", "Don't Use SLAM"])  # Add your specific options here
        self.btn_navigate = QPushButton('Navigate to New Point')       
        self.btn_navigate.clicked.connect(lambda: self.navigate_to_new_point())
        nav_layout = QVBoxLayout()
        nav_layout.addWidget(self.combo_navigate_options)
        nav_layout.addWidget(self.btn_navigate)
        layout.addLayout(nav_layout)

        back_btn = QPushButton('Return to Main Page')
        back_btn.clicked.connect(lambda: change_page_fn(0,True))
        layout.addWidget(back_btn)

        self.setLayout(layout)
        
    def show_help(self):
        help_text = "This is the 2D map construction page with Hector-SLAM algorithm.\n You can navigate around the room by clicking 'Navigate to New Point' button. \n Depending on your choice, it will start Hector-SLAM algorithm and you can use 'WASD' keys of your keyboard to move the Oedipus forward-left-backward-right. \n You can use 'X' to stop the motors and 'ESC' key to stop navigation. Wait until SLAM algorithm to finalize. \n Once the navigation is finished, it will save the 2D map generated by the SLAM, if 'Save' option is selected. \n 'Don't Use SLAM' option could be used to adjust the initial position of the robot, but when gathering data 'Save' option should be used. \n If 'Save' option is selected, a RVIZ page will be open to show the status and location of the robot\n You can return back to the main page by clicking 'Return to Main Page' button."
        dlg = HelpDialog(help_text)
        dlg.exec_()


    def navigate_to_new_point(self):
        # Simulate action       
        self.btn_navigate.setDisabled(True)
        selected_option = self.combo_navigate_options.currentText()

        if selected_option == "Save":
            run_in_new_terminal("python3 maps/motorControlWASD.py YS")
        elif selected_option == "Don't Save":
            run_in_new_terminal("python3 maps/motorControlWASD.py N")
        self.btn_navigate.setEnabled(True)


if __name__ == '__main__':
    app = QApplication(sys.argv)
    app.setStyleSheet(qdarkstyle.load_stylesheet_pyqt5())
    ex = App()
    ex.show()
    sys.exit(app.exec_())


