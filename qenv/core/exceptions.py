class NoImageException(Exception):
    """
    Exception raised for errors that occur when an image is expected but not provided.

    This can be used to signal issues such as a missing image file, an undefined image variable,
    or an invalid image path in image processing workflows.
    """

    def __init__(self, *args: object) -> None:
        super().__init__(*args)