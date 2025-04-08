# Bluetooth Discovery Project

This project is designed to discover nearby Bluetooth devices using Python. It utilizes the `pybluez` library to scan for devices and display their addresses and names.

## Project Structure

```
bluetooth-discovery-project
├── src
│   ├── main.py
│   └── bluetooth
│       └── scanner.py
├── requirements.txt
└── README.md
```

## Setup Instructions

1. **Clone the repository:**
   ```
   git clone <repository-url>
   cd bluetooth-discovery-project
   ```

2. **Install the required dependencies:**
   Make sure you have Python 3 installed. Then, run:
   ```
   pip install -r requirements.txt
   ```

## Usage

1. **Run the application:**
   Navigate to the `src` directory and execute:
   ```
   python main.py
   ```

2. **Discover Bluetooth devices:**
   The application will automatically start scanning for nearby Bluetooth devices and display their addresses and names in the console.

## Dependencies

- `pybluez`: A Python module for Bluetooth communication.

## License

This project is licensed under the MIT License. See the LICENSE file for more details.