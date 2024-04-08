# python imports
import os
import csv
from abc import abstractmethod


class DataWriter:
    """
    A utility class for recording data into a CSV file.

    This class provides a simple interface to record lists of data into a CSV file,
    which is stored in a specified directory. It is designed to be used for collecting
    training data or any other form of structured data that can be represented in CSV format.

    Attributes:
        data_path (str): The file path to the directory where the CSV file will be stored.
        csv_filepath (str): The file path to the CSV file where data will be recorded.
        counter (int): A counter to keep track of the number of data entries recorded
            (not implemented).

    Methods:
        setup(): Ensures that the directory for storing the CSV file exists, creating it
            if necessary.
        record_data(csv_data): Appends a list of data to the CSV file.
        execute(*args): An abstract method to be implemented by subclasses, defining the logic
            for data collection, execute in every iteration
    """

    def __init__(self, folder_name: str = 'training_data', csv_name: str = 'training_data.csv') -> None:
        current_path: str = os.getcwd()
        self.data_path: str = os.path.join(current_path, folder_name)
        self.csv_filepath: str = os.path.join(self.data_path, csv_name)
        self.counter: int = 0

    def setup(self) -> None:
        if not os.path.exists(self.data_path):
            os.makedirs(self.data_path)

    def record_data(self, csv_data: list) -> None:
        with open(self.csv_filepath, 'a', newline='') as csvfile:
            writer = csv.writer(csvfile)
            writer.writerow(csv_data)

    @abstractmethod
    def execute(self, *args) -> None:
        # Abstract method, to be implemented by subclasses.
        pass
