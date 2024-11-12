import sys
from PyQt5.QtWidgets import QApplication, QWidget
from PyQt5.QtGui import QPainter, QPen, QColor, QBrush
from PyQt5.QtCore import Qt

class PaintWidget(QWidget):
    def __init__(self):
        super().__init__()
        self.initUI()

    def initUI(self):
        self.setWindowTitle('2D Drawing Software')
        self.setGeometry(100, 100, 800, 600)

    def paintEvent(self, event):
        painter = QPainter(self)
        painter.begin(self)
        self.drawShapes(painter)
        painter.end()

    def drawShapes(self, painter):
        # 设置画笔颜色和宽度
        pen = QPen(Qt.black, 2, Qt.SolidLine)
        painter.setPen(pen)

        # 绘制直线
        painter.drawLine(50, 50, 200, 50)

        # 设置画刷颜色
        brush = QBrush(QColor(255, 0, 0))
        painter.setBrush(brush)

        # 绘制矩形
        painter.drawRect(50, 100, 100, 50)

        # 绘制椭圆
        painter.drawEllipse(300, 100, 100, 50)

        # 绘制多边形
        points = [QPoint(450, 150), QPoint(500, 100), QPoint(550, 150), QPoint(500, 200)]
        painter.drawPolygon(points)

if __name__ == '__main__':
    app = QApplication(sys.argv)
    widget = PaintWidget()
    widget.show()
    sys.exit(app.exec_())