ESP-IDF CI Build System
=======================
You can just folk this repo and update it with your project folder files and follow the steps below to execute.

# How to Run the Build on GitHub
Since we are managing everything directly via the browser, follow these steps to trigger the compilation:
- Open your repository on GitHub.
- Click on the Actions tab along the top menu bar.
- In the left sidebar, click on CI Build System (the name of our workflow).
- You will see a light-colored banner appear on the right side with a Run workflow dropdown button. Click it.
- A small configuration form will slide open:
    Target Microcontroller: Leave it as esp32 (or change it if you test on an S3/C3 variant later).
    Desired name: Type the name you want for your application binary (e.g., smart_tag_v1.0).
  
- Click the green Run workflow button.
- The page will refresh after a few seconds, showing a running job instance.

## Where to Find Your Compiled Files
- Once the running job finishes (it will display a green checkmark), click directly on the title of that specific run instance.
- Scroll all the way down to the bottom of the summary page to the Artifacts section.
- You will see a downloadable zip package named after your custom binary input (e.g., smart_tag_v1.0-binaries).
- Click to download it. When you extract it, you will have exactly your 3 required flashing targets:
    - bootloader.bin
    - partition-table.bin
    - smart_tag_v1.0.bin (or whatever custom name you provided)

## Testing and Comparing the Binaries
To ensure everything runs seamlessly when flashing with the official Espressif Flash Download Tool:
- Keep track of the memory offsets used during your local laptop build.
- Flash the 3 files generated locally first to confirm the base project firmware functions correctly.
- Erase the chip, flash the 3 files downloaded from the GitHub Action artifacts at those exact same offsets, and verify that the device behaves identically.
- This will guarantee that the toolchain versions match up nicely and give you full confidence in your remote build setup.


  ## Credits
  
  - **Developer:**
    - Eric Mulwa
    - Date: 12/06/2026
  ---
