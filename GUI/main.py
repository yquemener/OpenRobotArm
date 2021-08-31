import pickle

import PySimpleGUI as sg
import serial
ser = serial.Serial('/dev/ttyACM0', 57600)

sg.theme('DarkAmber')   # Add a touch of color

banks = dict()
banks_names = ('M1', 'M2', 'M3', 'M4')

input_layout = [[sg.Text('Base'), sg.Slider(orientation='h', key='BASE', range=(0, 180), default_value=0, enable_events=True)],
                [sg.Text('Shoulder'), sg.Slider(orientation='h', key='SHOULDER', range=(15, 165), default_value=40, enable_events=True)],
                [sg.Text('Elbow'), sg.Slider(orientation='h', key='ELBOW', range=(0, 180), default_value=180, enable_events=True)],
                [sg.Text('Wrist vert'), sg.Slider(orientation='h', key='WRISTV', range=(0, 180), default_value=170, enable_events=True)],
                [sg.Text('Wrist rot'), sg.Slider(orientation='h', key='WRISTR', range=(0, 180), default_value=0, enable_events=True)],
                [sg.Text('Gripper'), sg.Slider(orientation='h', key='GRIPPER', range=(10, 73), default_value=73, enable_events=True)],
                [sg.Button('Run'),sg.Checkbox('Autorun', key='autorun'), sg.Button('Save'),sg.Button('Load'),],
                [sg.Radio('M1', "bank", k="M1", enable_events=True), sg.Radio('M2', "bank", k="M2", enable_events=True),
                 sg.Radio('M3', "bank", k="M3", enable_events=True), sg.Radio('M4', "bank", k="M4", enable_events=True)]]
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
    return values["BASE"], values["SHOULDER"], values["ELBOW"], values["WRISTV"], values["WRISTR"], values["GRIPPER"]


def set_tuple(window, t):
    keys = ["BASE", "SHOULDER", "ELBOW", "WRISTV", "WRISTR", "GRIPPER"]
    for k, v in zip(keys, t):
        window[k].update(v)


def get_Selected_bank(values):
    for k in banks_names:
        if values[k]:
            return k
    return None


if __name__ == '__main__':
    banks = pickle.load(open("banks.pickle", "rb"))
    window = sg.Window('Window Title', input_layout)
    while True:
        event, values = window.read(100)
        if event == sg.WIN_CLOSED or event == 'Cancel':  # if user closes window or clicks cancel
            break
        if event == "Run" or values['autorun']:
            print(values)
            b = bytes([255] + [int(x) for x in get_tuple(values)])
            ser.write(b)
            ser.flush()
        if event == "Save":
            selected_bank = get_Selected_bank(values)
            if selected_bank is not None:
                banks[selected_bank]=get_tuple(values)
            pickle.dump(banks, open("banks.pickle", "wb"))
        if event == "Load":
            selected_bank = get_Selected_bank(values)
            if selected_bank is not None:
                set_tuple(window, banks[selected_bank])

        #     print(b)
        #     print("_")
        #     for i in range(4):
        #         c = ser.read()
        #         print(c)
        print('Event ', event)

