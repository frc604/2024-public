# 2024 Compbot Code

2024 Compbot onboard and offboard code for FRC604.

## Setup

### Install Python dependencies

- [Install](https://www.python.org/downloads/) Python 3.11 if it is not yet installed

- Run `pip3 install -r requirements.txt`

### To enable autoformatting when saving a file:

- [Install](https://marketplace.visualstudio.com/items?itemName=richardwillis.vscode-spotless-gradle) the `richardwillis.vscode-spotless-gradle` VS Code extension.

- In `File > Preferences > Settings`, search for `Format on Save` and enable it.

- If asked to select a formatter, choose `Spotless Gradle`.

## Code Structure

- `src/main/java/frc/quixlib`: General utilities to be used year-after-year
- `src/main/java/frc/robot`: Year-specific code
- `src/test/java/frc/...`: Unit tests for both `quixlib` and `robot`
- `offboard`: Code that doesn't run onboard the robot

## Commands

- `./gradlew spotlessApply` to format all code.
- `python3 offboard/quixpf/quixsam.py --local --view3d` to run the localizer in simulation
- View simulated cameras at:
    - http://localhost:1182 (Front left)
    - http://localhost:1184 (Front right)
    - http://localhost:1186 (Back left)
    - http://localhost:1188 (Back right)
