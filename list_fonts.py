from PyQt6.QtWidgets import QApplication
from PyQt6.QtGui import QFontDatabase
import sys

app = QApplication(sys.argv)
db = QFontDatabase()
fonts = db.families()

print("Fuentes disponibles:")
print("-------------------")
for font in fonts:
    print(f"• {font}")
