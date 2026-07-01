/*
 * MIT License
 *
 * Copyright (c) 2026 Adrian Port
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <ft2build.h>
#include FT_FREETYPE_H

#define WIDTH 480
#define HEIGHT 120
#define TOTAL_BYTES (WIDTH * HEIGHT)

unsigned char bitmap[TOTAL_BYTES];

// background tone (dark bluish)
#define BG_R 20
#define BG_G 30
#define BG_B 50

// clamp helper
int clamp(int v, int min, int max) {
    if (v < min) return min;
    if (v > max) return max;
    return v;
}

// convert RGB888 → RGB332
unsigned char rgb332(int r, int g, int b) {
    return ((r >> 5) << 5) | ((g >> 5) << 2) | (b >> 6);
}

void set_pixel(int x, int y, int r, int g, int b) {
    if (x < 0 || x >= WIDTH || y < 0 || y >= HEIGHT) return;

    int idx = y * WIDTH + x;

    unsigned char new_val = rgb332(r, g, b);

    // simple overwrite (foreground dominates naturally)
    bitmap[idx] = new_val;
}

void draw_pixel_thick(int x, int y, int thickness, int r, int g, int b) {
    for (int dy = -thickness; dy <= thickness; dy++) {
        for (int dx = -thickness; dx <= thickness; dx++) {
            set_pixel(x + dx, y + dy, r, g, b);
        }
    }
}

int main() {
    FT_Library ft;
    FT_Face face;

    const char *text = "VH-XVG";
    const char *font_path = "assets/fonts/cinzel/Cinzel-Regular.ttf";

    int outline = 1;
    int bold = 1;
    int sts;

    FT_Init_FreeType(&ft);
    sts = FT_New_Face(ft, font_path, 0, &face);
    if (sts) {
      printf("FT_New_Face failed, sts %d (%s)\n", sts, FT_Error_String(sts));
      exit(0);
      }


    // --- fill background ---
    for (int i = 0; i < TOTAL_BYTES; i++) {
        bitmap[i] = rgb332(BG_R, BG_G, BG_B);
    }

    // --- AUTO SCALE ---
    int best_size = 10;

    for (int size = 10; size < 200; size++) {
        FT_Set_Pixel_Sizes(face, 0, size);

        int width = 0, max_top = 0, max_bottom = 0;

        for (int i = 0; text[i]; i++) {
            FT_Load_Char(face, text[i], FT_LOAD_RENDER);

            FT_GlyphSlot g = face->glyph;

            width += g->advance.x >> 6;

            if (g->bitmap_top > max_top) max_top = g->bitmap_top;

            int bottom = g->bitmap.rows - g->bitmap_top;
            if (bottom > max_bottom) max_bottom = bottom;
        }

        int height = max_top + max_bottom;

        if (width > WIDTH - 20 || height > HEIGHT - 20)
            break;

        best_size = size;
    }

    FT_Set_Pixel_Sizes(face, 0, best_size);

    // --- MEASURE ---
    int total_width = 0, max_top = 0, max_bottom = 0;

    for (int i = 0; text[i]; i++) {
        FT_Load_Char(face, text[i], FT_LOAD_RENDER);

        FT_GlyphSlot g = face->glyph;

        total_width += g->advance.x >> 6;

        if (g->bitmap_top > max_top) max_top = g->bitmap_top;

        int bottom = g->bitmap.rows - g->bitmap_top;
        if (bottom > max_bottom) max_bottom = bottom;
    }

    int pen_x = (WIDTH - total_width) / 2;
    int pen_y = (HEIGHT + max_top - max_bottom) / 2;

    // --- RENDER ---
    for (int i = 0; text[i]; i++) {
        FT_Load_Char(face, text[i], FT_LOAD_RENDER);

        FT_GlyphSlot g = face->glyph;
        int bold_offset = bold ? 1 : 0;

        for (int row = 0; row < g->bitmap.rows; row++) {
            for (int col = 0; col < g->bitmap.width; col++) {

                unsigned char gray =
                    g->bitmap.buffer[row * g->bitmap.pitch + col];

                if (gray > 0) {
                    int x = pen_x + g->bitmap_left + col;
                    int y = pen_y - g->bitmap_top + row;

                    // --- grayscale → bluish RGB ---
                    int r = clamp((int)(gray * 0.7), 0, 255);
                    int gcol = clamp((int)(gray * 0.8), 0, 255);
                    int b = clamp((int)(gray * 1.0), 0, 255);

                    if (outline)
                        draw_pixel_thick(x, y, outline, r, gcol, b);

                    set_pixel(x, y, r, gcol, b);
                    set_pixel(x + bold_offset, y, r, gcol, b);
                }
            }
        }

        pen_x += (g->advance.x >> 6) + bold_offset;
    }

    // --- RAW OUTPUT ---
    FILE *f = fopen("image_rgb332.raw", "wb");
    fwrite(bitmap, 1, TOTAL_BYTES, f);
    fclose(f);

    // --- HEADER ---
    FILE *h = fopen("image_rgb332.h", "w");

    fprintf(h, "#ifndef IMAGE_RGB332_H\n#define IMAGE_RGB332_H\n\n");
    fprintf(h, "#define IMG_WIDTH %d\n", WIDTH);
    fprintf(h, "#define IMG_HEIGHT %d\n", HEIGHT);
    fprintf(h, "#define IMG_SIZE %d\n\n", TOTAL_BYTES);

    fprintf(h, "const unsigned char image_data[%d] = {\n", TOTAL_BYTES);

    for (int i = 0; i < TOTAL_BYTES; i++) {
        if (i % 16 == 0) fprintf(h, "  ");
        fprintf(h, "0x%02X,", bitmap[i]);
        if (i % 16 == 15) fprintf(h, "\n");
    }

    fprintf(h, "\n};\n\n#endif\n");
    fclose(h);

    FT_Done_Face(face);
    FT_Done_FreeType(ft);

    printf("Generated RGB332 image (%d bytes)\n", TOTAL_BYTES);
    return 0;
}
