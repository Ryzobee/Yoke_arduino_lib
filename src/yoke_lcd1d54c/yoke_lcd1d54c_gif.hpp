#pragma once

#include <AnimatedGIF.h>
#include <climits>
#include "yoke_lcd1d54c.hpp"

class Yoke_lcd1d54c_gif {
public:
    explicit Yoke_lcd1d54c_gif(Yoke_lcd1d54c& lcd) : _lcd(lcd) {}
    ~Yoke_lcd1d54c_gif() { close(); }
    Yoke_lcd1d54c_gif(const Yoke_lcd1d54c_gif&) = delete;
    Yoke_lcd1d54c_gif& operator=(const Yoke_lcd1d54c_gif&) = delete;

    /**
     * @brief Open an in-memory GIF and clear the initialized display to black.
     *
     * @param data GIF bytes; must remain valid until close() or the next open().
     * @param size GIF data length in bytes, from 1 to INT_MAX.
     * @return Whether the GIF was opened successfully.
     */
    bool open(const uint8_t* data, size_t size) {
        close();
        if (!_lcd.isInitialized() || data == nullptr || size == 0 || size > INT_MAX) {
            return false;
        }
        _gif.begin(BIG_ENDIAN_PIXELS);
        _opened = _gif.open(const_cast<uint8_t*>(data), static_cast<int>(size), drawGifLine) != 0;
        if (_opened) {
            _lcd.fillScreen(TFT_BLACK);
        }
        return _opened;
    }

    /**
     * @brief Close the current animation without changing the screen contents.
     */
    void close() {
        if (_opened) {
            _gif.close();
            _opened = false;
        }
    }

    /**
     * @brief Draw one frame, wait for its frame delay, and rewind at the end.
     *
     * @return True on successful playback, false if unavailable or decoding failed.
     */
    bool update() {
        if (!_opened || !_lcd.isInitialized()) {
            return false;
        }
        _lcd.startWrite();
        int result = _gif.playFrame(true, nullptr, this);
        _lcd.endWrite();
        if (result < 0) {
            close();
            return false;
        }
        if (result == 0) {
            _gif.reset();
        }
        return true;
    }

    bool isOpen() const { return _opened; }
    int width() { return _opened ? _gif.getCanvasWidth() : 0; }
    int height() { return _opened ? _gif.getCanvasHeight() : 0; }

private:
    /**
     * @brief Route a decoder scanline to the owning player instance.
     *
     * @param draw Decoded indexed pixels and palette supplied by AnimatedGIF.
     */
    static void drawGifLine(GIFDRAW* draw) {
        if (draw != nullptr && draw->pUser != nullptr) {
            static_cast<Yoke_lcd1d54c_gif*>(draw->pUser)->drawLine(draw);
        }
    }

    /**
     * @brief Clip and draw a scanline, preserving transparent pixel runs.
     *
     * @param draw Decoded scanline with GIF transparency and disposal settings.
     */
    void drawLine(GIFDRAW* draw) {
        if (!_lcd.isInitialized()) return;
        int y = draw->iY + draw->y;
        int left = std::max(0, -draw->iX);
        int right = std::min<int>(draw->iWidth, _lcd.width() - draw->iX);
        if (y < 0 || y >= _lcd.height() || left >= right) return;

        bool restoreBackground = draw->ucDisposalMethod == 2;
        int start = left;
        while (start < right) {
            if (draw->ucHasTransparency && !restoreBackground &&
                draw->pPixels[start] == draw->ucTransparent) {
                ++start;
                continue;
            }
            int count = 0;
            while (start + count < right && count < YOKE_LCD1D54C_SIZE) {
                uint8_t pixel = draw->pPixels[start + count];
                if (draw->ucHasTransparency && pixel == draw->ucTransparent) {
                    if (!restoreBackground) break;
                    pixel = draw->ucBackground;
                }
                _lineBuffer[count++] = draw->pPalette[pixel];
            }
            _lcd.pushImage(draw->iX + start, y, count, 1, _lineBuffer);
            start += count;
        }
    }

    Yoke_lcd1d54c& _lcd;
    AnimatedGIF _gif;
    uint16_t _lineBuffer[YOKE_LCD1D54C_SIZE];
    bool _opened = false;
};
