from qvl.qlabs import QuanserInteractiveLabs

def connect_to_qlab() -> None:
    qlabs = QuanserInteractiveLabs()
    qlabs.open("localhost")