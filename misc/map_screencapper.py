import pyautogui
import time
import os
# from PIL import ImageGrab
# from functools import partial

timeBetweenScreenshots = 30 # in seconds
screenshotPath = '/home/suh/Pictures/Testing/06/mexplore'
screenshotPrefix = '06-mexplore'

xStart = 1965
width = 2560-(xStart - 1920)
yStart = 30
height = 1440-yStart

#xStart = 41
#width = 1920-(xStart - 0)
#yStart = 30
#height = 1080-yStart
i = 1

if not os.path.exists(screenshotPath):
    print(f"Path does not exist: {screenshotPath}")
    exit(1)
time.sleep(2)
while True:
    timePassed = (i-1) * timeBetweenScreenshots

    print(f'Grabbing sreenshot {screenshotPrefix}#{i}')
    filePath = f'{screenshotPath}/{screenshotPrefix}-{int(timePassed / 60)}m{int(timePassed % 60)}s'

    while os.path.exists(f'{filePath}.png'):
        print(f'Screenshot path "{filePath}" already exists, adding -final to screenshot name')
        filePath = f'{filePath}-final'

    # ImageGrab.grab = partial(ImageGrab.grab, all_screens=True)

    myScreenshot = pyautogui.screenshot(region=(xStart, yStart, width, height))
    myScreenshot.save(f'{filePath}.png')
    i += 1

    time.sleep(timeBetweenScreenshots)
