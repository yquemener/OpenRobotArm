import sys
import pickle

import PySimpleGUI as sg
import serial

success = False
num = 0
while not success:
    try:
        ser = serial.Serial(f'/dev/ttyACM{num}', 57600)
        success=True
    except:
        print(f"Failed to connect to /dev/ttyACM{num}")
        num+=1
        if num > 100:
            sys.exit()


sg.theme('DarkAmber')   # Add a touch of color

banks = dict()
banks_names = ('M1', 'M2', 'M3', 'M4', 'M5', 'M6')

input_layout = [[sg.Text('Base'), sg.Slider(orientation='h', key='BASE', range=(0, 180), default_value=0)],
                [sg.Text('Shoulder'), sg.Slider(orientation='h', key='SHOULDER', range=(15, 165), default_value=40)],
                [sg.Text('Elbow'), sg.Slider(orientation='h', key='ELBOW', range=(0, 180), default_value=180)],
                [sg.Text('Wrist vert'), sg.Slider(orientation='h', key='WRISTV', range=(0, 180), default_value=170)],
                [sg.Text('Wrist rot'), sg.Slider(orientation='h', key='WRISTR', range=(0, 180), default_value=0)],
                # [sg.Text('Gripper'), sg.Slider(orientation='h', key='GRIPPER', range=(10, 73), default_value=73)],
                [sg.Text('Gripper'), sg.Slider(orientation='h', key='GRIPPER', range=(0, 255), default_value=73)],
                [sg.Text('Pump'), sg.Slider(orientation='h', key='PUMP', range=(0, 1), default_value=0)],
                [sg.Text('Valve'), sg.Slider(orientation='h', key='VALVE', range=(0, 1), default_value=0)],
                [sg.Button('Start'),sg.Button('Send'),sg.Checkbox('Autosend', key='autosend'), sg.Button('Save'),sg.Button('Load'),],
                [sg.Radio('M1', "bank", k="M1", enable_events=True), sg.Radio('M2', "bank", k="M2", enable_events=True),
                 sg.Radio('M3', "bank", k="M3", enable_events=True), sg.Radio('M4', "bank", k="M4", enable_events=True),
                 sg.Radio('M5', "bank", k="M5", enable_events=True), sg.Radio('M6', "bank", k="M6", enable_events=True)]]
                #
                # [sg.Input(key='-INPUT-')],
                # [sg.Slider(orientation='h', key='-SKIDER-'),
                #  sg.Image(data=sg.DEFAULT_BASE64_LOADING_GIF, enable_events=True, key='-GIF-IMAGE-'), ],
                # [sg.Checkbox('Checkbox', default=True, k='-CB-')],
                # [sg.Radio('Radio1', "RadioDemo", default=True, size=(10, 1), k='-R1-'),
                #  sg.Radio('Radio2', "RadioDemo", default=True, size=(10, 1), k='-R2-')],
                # [sg.Combo(values=('Combo 1', 'Combo 2', 'Combo 3'), default_value='Combo 1', readonly=True,
                #           k='-COMBO-'),
                #  sg.OptionMenu(values=('Option 1', 'Option 2', 'Option 3'), k='-OPTION MENU-'), ],
                # [sg.Spin([i for i in range(1, 11)], initial_value=10, k='-SPIN-'), sg.Text('Spin')],
                # [sg.Multiline(
                #     'Demo of a Multi-Line Text Element!\nLine 2\nLine 3\nLine 4\nLine 5\nLine 6\nLine 7\nYou get the point.',
                #     size=(45, 5), k='-MLINE-')],
                # [sg.Button('Button'), sg.Button('Popup'), sg.Button(image_data=sg.DEFAULT_BASE64_ICON, key='-LOGO-')]]


def get_tuple(values):
    return values["BASE"], values["SHOULDER"], values["ELBOW"], values["WRISTV"], values["WRISTR"], values["GRIPPER"], values["PUMP"], values["VALVE"]


def set_tuple(window, t):
    keys = ["BASE", "SHOULDER", "ELBOW", "WRISTV", "WRISTR", "GRIPPER", "PUMP", "VALVE"]
    for k, v in zip(keys, t):
        window[k].update(v)


def get_selected_bank(values):
    for k in banks_names:
        if values[k]:
            return k
    return None


def get_amps():
    ser.write(bytes([255, ord('R')]))
    ser.flush()
    c = ser.read(1)
    s = bytes("", "ascii")
    while c not in (b'\n', b'\r'):
        s += c
        c = ser.read(1)
    ser.flushInput()
    print(s)


if __name__ == '__main__':
    old_values = None
    banks = pickle.load(open("banks.pickle", "rb"))
    window = sg.Window('Braccio control', input_layout)
    event, values = window.read(100)
    if "M1" in banks.keys():
        set_tuple(window, banks["M1"])

    while True:
        event, values = window.read(100)

        if event == sg.WIN_CLOSED or event == 'Cancel':  # if user closes window or clicks cancel
            break
        if event == "Send" or (event == "__TIMEOUT__" and values['autosend']):
            get_amps()
            if ser.out_waiting == 0:
                if old_values != values:
                    b = bytes([255, ord('G')] + [int(x) for x in get_tuple(values)])
                    print(get_tuple(values))
                    ser.write(b)
                    ser.flush()
                    old_values=values
        if event == "Start":
            if ser.out_waiting == 0:
                b = bytes([255, ord('S')])
                ser.write(b)
                ser.flush()
        if event == "Save":
            selected_bank = get_selected_bank(values)
            if selected_bank is not None:
                banks[selected_bank]=get_tuple(values)
            pickle.dump(banks, open("banks.pickle", "wb"))
        if event == "Load":
            selected_bank = get_selected_bank(values)
            if selected_bank is not None:
                set_tuple(window, banks[selected_bank])

        #     print(b)
        #     print("_")
        #     for i in range(4):
        #         c = ser.read()
        #         print(c)
        # print('Event ', event)

