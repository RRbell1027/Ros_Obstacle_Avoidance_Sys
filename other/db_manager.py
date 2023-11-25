import sqlite3
import os

DB_FILE = 'other/car_database.db'

def create_db():
    """建立資料庫及相應表格的函數"""
    conn = sqlite3.connect(DB_FILE)
    c = conn.cursor()

    # CAR_ID
    c.execute('''CREATE TABLE IF NOT EXISTS CAR_ID
                 (id INTEGER PRIMARY KEY AUTOINCREMENT,
                  car_id INTEGER UNIQUE)''')

    # SENSOR_DATA
    c.execute('''CREATE TABLE IF NOT EXISTS SENSOR_DATA
                 (id INTEGER PRIMARY KEY AUTOINCREMENT,
                  car_id INTEGER,
                  time_stamp TEXT,
                  sensor_data TEXT,
                  FOREIGN KEY (car_id) REFERENCES car_id (id))''')

    conn.commit()
    conn.close()

def add_car(car_id):
    """管理資料庫的函數，可以用來查詢、插入或更新資料"""
    conn = sqlite3.connect(DB_FILE)
    c = conn.cursor()

    c.execute("INSERT INTO CAR_ID (car_id) VALUES (?)", (car_id))

    conn.commit()
    conn.close()

def add_data(car_id, time_stamp, sensor_data):
    conn = sqlite3.connect(DB_FILE)
    c = conn.cursor()

    c.execute("INSERT INTO SENSOR_DATA (car_id, time_stamp, sensor_data) VALUES (?, ?, ?)",
              (car_id, time_stamp, sensor_data))

    conn.commit()
    conn.close()

if __name__ == '__main__':
    create_db()