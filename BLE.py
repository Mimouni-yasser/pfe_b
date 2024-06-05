import asyncio
import bleak
from bleak import BleakClient, BleakScanner
import time
from PyQt5.QtWidgets import (QApplication, QMainWindow, QTableWidget, 
                             QTableWidgetItem, QPushButton, QVBoxLayout, 
                             QWidget, QHBoxLayout, QLineEdit, QLabel, QFormLayout, QMessageBox, QComboBox)

from PyQt5.QtCore import QTimer, Qt
import sys

CHARA_UUID = "00000055-0000-1000-8000-00805f9b34fb"
SRVC_UUID = "00000080-0000-1000-8000-00805f9b34fb"

async def find_device():
    mac = None
    found = False
    while mac is None:
        devices = await BleakScanner.discover()
        for d in devices:
            print(d)
            if d.name == "NRF52840":
                mac = d.address
                found = True
        if found: 
            return mac
        else: 
            await asyncio.sleep(0.5)

async def scan_all():
    print("scanning")
    devices = await BleakScanner.discover()
    return devices

def bitstring_to_bytes(s):
    return int(s, 2).to_bytes((len(s) + 7) // 8, byteorder='big')


class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Device Scanner")
        
        self.table = QTableWidget()
        self.button = QPushButton("Scan")
        
        self.interface_type = QComboBox()
        self.interface_type.addItems(["", "i2c", "GPIO"])
        self.pin = QLineEdit()
        self.i2c_address = QLineEdit()
        self.registers_1 = QLineEdit()
        self.registers_2 = QLineEdit()
        self.id_input = QLineEdit()
        self.send_config_button = QPushButton("Send Config")
        
        form_layout = QFormLayout()
        form_layout.addRow(QLabel("Sensor ID"), self.id_input)
        form_layout.addRow(QLabel("Interface Type:"), self.interface_type)
        form_layout.addRow(QLabel("Pin:"), self.pin)
        form_layout.addRow(QLabel("I2C Address:"), self.i2c_address)
        form_layout.addRow(QLabel("Registers 1:"), self.registers_1)
        form_layout.addRow(QLabel("Registers 2:"), self.registers_2)
        form_layout.addWidget(self.send_config_button)
        
        form_container = QWidget()
        form_container.setLayout(form_layout)
        
        left_layout = QVBoxLayout()
        left_layout.addWidget(self.table)
        left_layout.addWidget(self.button)
        
        left_container = QWidget()
        left_container.setLayout(left_layout)
        
        main_layout = QHBoxLayout()
        main_layout.addWidget(left_container)
        main_layout.addWidget(form_container)
        
        container = QWidget()
        container.setLayout(main_layout)
        
        self.setCentralWidget(container)
        
        self.button.clicked.connect(self.scan_bt_cb)
        self.send_config_button.clicked.connect(self.send_config_cb)
        
        # Set up a QTimer to periodically call asyncio tasks
        self.timer = QTimer()
        self.timer.timeout.connect(self.process_asyncio_events)
        self.timer.start(50)  # Adjust the interval as needed

    def update_table(self, devices):
        self.table.setRowCount(len(devices))
        self.table.setColumnCount(2)
        self.table.setHorizontalHeaderLabels(["Name", "Address"])
        for row, device in enumerate(devices):
            name_item = QTableWidgetItem(device.name)
            address_item = QTableWidgetItem(device.address)
            if device.name == "NRF52840":
                name_item.setBackground(Qt.red)
                name_item.setForeground(Qt.white)
                address_item.setBackground(Qt.red)
                address_item.setForeground(Qt.white)
            self.table.setItem(row, 0, name_item)
            self.table.setItem(row, 1, address_item)

    def scan_bt_cb(self):
        self.button.setText("Scanning...")
        self.button.setEnabled(False)
        asyncio.ensure_future(self.scan_and_update_table())

    async def scan_and_update_table(self):
        devices = await scan_all()
        self.update_table(devices)
        self.button.setText("Scan")
        self.button.setEnabled(True)

    def send_config_cb(self):
        try:
            self.send_config_button.setEnabled(False)
            sel = self.table.selectedItems()[0].row()
            dev = self.table.item(sel, 1).text()
        except Exception as e:
            QMessageBox.warning(self, "No Device Selected", "Please select a device from the table.")
            print(e)
            self.send_config_button.setEnabled(True)
            return
        id = self.id_input.text()
        interface_type = self.interface_type.currentText()
        pin = self.pin.text()
        i2c_address = self.i2c_address.text()
        registers_1 = self.registers_1.text()
        registers_2 = self.registers_2.text()
        if any(v == '' or v is None for v in [interface_type, pin, i2c_address, registers_1, registers_2]):
            QMessageBox.warning(self, "Missing Values", "Please fill in all the fields.")
            return
        print(id, interface_type, pin, i2c_address, registers_1, registers_2)
        
        
        msg = ""
        
        id_bin = bin(int(id))[2:].zfill(8)
        msg = msg + str(id_bin)
        
        if interface_type == "i2c":
            msg = msg + "01"
        elif interface_type == "GPIO":
            msg = msg + "00"
        else:
            msg = msg + "11"
        
        i2c_address_bin = bin(int(i2c_address, 16))[2:].zfill(8)
        msg = msg + str(i2c_address_bin)
        
        registers_1_bin = bin(int(registers_1))[2:].zfill(8)
        msg = msg + str(registers_1_bin)

        registers_2_bin = bin(int(registers_2))[2:].zfill(8)
        msg = msg + str(registers_2_bin)
        
        buf =  bitstring_to_bytes(msg)
        print(buf)
        #self.send_config_button.setEnabled(False)
        asyncio.ensure_future(self.send_to_mac(dev, 5, 5))
        
    async def send_to_mac(self, mac, buf, retries):
        try_n = 0
        if mac is not None:
            ok = False
            while not ok:
                try:
                    if try_n > retries: 
                        return -20
                    async with BleakClient(mac) as client:
                        s = client.services.get_service(SRVC_UUID)
                        k = s.get_characteristic(CHARA_UUID)
                        print(k)
                        ok = True
                        print(k.max_write_without_response_size)
                        await client.write_gatt_char(k, buf, True)
                        self.send_config_button.setEnabled(True)
                        return 0
                except bleak.exc.BleakDeviceNotFoundError as e:
                    try_n += 1
                    print("not found")
                    ok = False

    def process_asyncio_events(self):
        loop = asyncio.get_event_loop()
        loop.stop()
        loop.run_forever()

if __name__ == "__main__":
    app = QApplication(sys.argv)
    
    loop = asyncio.new_event_loop()
    asyncio.set_event_loop(loop)
    
    main_window = MainWindow()
    main_window.show()
    
    sys.exit(app.exec_())
