import os
import json
import webview
from webview.window import Window


class Api:
    def __init__(self) -> None:
        self.data: dict = {}
        self.file_path: str = os.getcwd() + "/qenv/gui/json/setting.json"
        self.window: Window = None

    def apply_setting(self, data) -> None:
        self.data = data
        with open(self.file_path, "w") as json_file:
            json.dump(self.data, json_file)

        print("Settings applied, please restart the program")
        self.window.destroy()
        os._exit(0)

    def load_json(self) -> dict:
        try:
            if os.path.exists(self.file_path):
                with open(self.file_path, "r") as json_file:
                    self.data = json.load(json_file)
        except:
            print("file is corrupted")
            return;

        return {'data': self.data}


class InitUI:
    def __init__(self) -> None:
        self.api: Api = Api()

    def initialize(self) -> None:
        html_path: str = os.getcwd()
        window: Window = webview.create_window(
            title='QCar Setting',
            url=html_path + '/qenv/gui/web/index.html',
            js_api=self.api,
            width=650,
            height=750,
        )

        self.api.window = window
        webview.start()

if __name__ == '__main__':
    ui = InitUI()
    ui.initialize()

    print(ui.api.data)