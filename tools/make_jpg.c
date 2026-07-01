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
#include <stdint.h>
#include <string.h>
#include <jpeglib.h>
#include <ft2build.h>
#include FT_FREETYPE_H

//#include "cinzel_regular_ttf.h"
#include "cinzel_bold_ttf.h"

#define WIDTH 480
#define HEIGHT 120
#define CHANNELS 3
#define IMAGE_BYTES (WIDTH * HEIGHT * CHANNELS)

static unsigned char image_rgb[IMAGE_BYTES];

/* Background tone similar to the source image */
#define BG_R 132
#define BG_G 130
#define BG_B 133

/* Main text tone */
#define FG_R 4
#define FG_G 4
#define FG_B 250

/* Outline tone */
#define OL_R 40
#define OL_G 40
#define OL_B 252

static void clear_image(void) {
    for (int y = 0; y < HEIGHT; y++) {
        for (int x = 0; x < WIDTH; x++) {
            int i = (y * WIDTH + x) * 3;
            image_rgb[i + 0] = BG_R;
            image_rgb[i + 1] = BG_G;
            image_rgb[i + 2] = BG_B;
        }
    }
}

static void blend_pixel(int x, int y, int alpha, int sr, int sg, int sb) {
    if (x < 0 || x >= WIDTH || y < 0 || y >= HEIGHT) {
        return;
    }
    if (alpha <= 0) {
        return;
    }
    if (alpha > 255) {
        alpha = 255;
    }

    int i = (y * WIDTH + x) * 3;

    int dr = image_rgb[i + 0];
    int dg = image_rgb[i + 1];
    int db = image_rgb[i + 2];

    image_rgb[i + 0] = (unsigned char)((sr * alpha + dr * (255 - alpha)) / 255);
    image_rgb[i + 1] = (unsigned char)((sg * alpha + dg * (255 - alpha)) / 255);
    image_rgb[i + 2] = (unsigned char)((sb * alpha + db * (255 - alpha)) / 255);
}

static void draw_thick_pixel(int x, int y, int radius, int alpha, int r, int g, int b) {
    for (int dy = -radius; dy <= radius; dy++) {
        for (int dx = -radius; dx <= radius; dx++) {
            blend_pixel(x + dx, y + dy, alpha, r, g, b);
        }
    }
}

static int write_jpeg(const char *filename, int quality) {
    struct jpeg_compress_struct cinfo;
    struct jpeg_error_mgr jerr;

    FILE *f = fopen(filename, "wb");
    if (!f) {
        perror(filename);
        return -1;
    }

    cinfo.err = jpeg_std_error(&jerr);
    jpeg_create_compress(&cinfo);
    jpeg_stdio_dest(&cinfo, f);

    cinfo.image_width = WIDTH;
    cinfo.image_height = HEIGHT;
    cinfo.input_components = 3;
    cinfo.in_color_space = JCS_RGB;

    jpeg_set_defaults(&cinfo);
    jpeg_set_quality(&cinfo, quality, 1);

    jpeg_start_compress(&cinfo, 1);

    while (cinfo.next_scanline < cinfo.image_height) {
        JSAMPROW row = &image_rgb[cinfo.next_scanline * WIDTH * 3];
        jpeg_write_scanlines(&cinfo, &row, 1);
    }

    jpeg_finish_compress(&cinfo);
    jpeg_destroy_compress(&cinfo);
    fclose(f);
    return 0;
}

static int measure_text(FT_Face face, const char *text, int pixel_size,
                        int *out_width, int *out_ascent, int *out_descent) {
    FT_Error err = FT_Set_Pixel_Sizes(face, 0, (FT_UInt)pixel_size);
    if (err) {
        return -1;
    }

    int width = 0;
    int ascent = 0;
    int descent = 0;

    FT_UInt prev = 0;
    int use_kerning = FT_HAS_KERNING(face);

    for (const char *p = text; *p; p++) {
        FT_ULong ch = (unsigned char)*p;
        FT_UInt glyph_index = FT_Get_Char_Index(face, ch);

        if (use_kerning && prev && glyph_index) {
            FT_Vector delta;
            if (FT_Get_Kerning(face, prev, glyph_index, FT_KERNING_DEFAULT, &delta) == 0) {
                width += delta.x >> 6;
            }
        }

        err = FT_Load_Char(face, ch, FT_LOAD_RENDER);
        if (err) {
            return -1;
        }

        FT_GlyphSlot g = face->glyph;
        width += g->advance.x >> 6;

        if (g->bitmap_top > ascent) {
            ascent = g->bitmap_top;
        }

        int glyph_descent = (int)g->bitmap.rows - g->bitmap_top;
        if (glyph_descent > descent) {
            descent = glyph_descent;
        }

        prev = glyph_index;
    }

    *out_width = width;
    *out_ascent = ascent;
    *out_descent = descent;
    return 0;
}

static int find_best_size(FT_Face face, const char *text) {
    const int pad_x = 12;
    const int pad_y = 10;
    int best = 8;

    for (int size = 8; size <= 200; size++) {
        int w, a, d;
        if (measure_text(face, text, size, &w, &a, &d) != 0) {
            break;
        }

        int h = a + d;
        if (w > (WIDTH - 2 * pad_x) || h > (HEIGHT - 2 * pad_y)) {
            break;
        }

        best = size;
    }

    return best;
}

static int render_text(FT_Face face, const char *text, int outline_px, int bold_px) {
    int best_size = find_best_size(face, text);
    if (best_size <= 0) {
        fprintf(stderr, "Could not determine font size\n");
        return -1;
    }

    int total_width, ascent, descent;
    if (measure_text(face, text, best_size, &total_width, &ascent, &descent) != 0) {
        fprintf(stderr, "Could not measure text\n");
        return -1;
    }

    if (FT_Set_Pixel_Sizes(face, 0, (FT_UInt)best_size) != 0) {
        fprintf(stderr, "FT_Set_Pixel_Sizes failed\n");
        return -1;
    }

    int pen_x = (WIDTH - total_width) / 2;
    int baseline_y = (HEIGHT + ascent - descent) / 2;

    FT_UInt prev = 0;
    int use_kerning = FT_HAS_KERNING(face);

    for (const char *p = text; *p; p++) {
        FT_ULong ch = (unsigned char)*p;
        FT_UInt glyph_index = FT_Get_Char_Index(face, ch);

        if (use_kerning && prev && glyph_index) {
            FT_Vector delta;
            if (FT_Get_Kerning(face, prev, glyph_index, FT_KERNING_DEFAULT, &delta) == 0) {
                pen_x += delta.x >> 6;
            }
        }

        if (FT_Load_Char(face, ch, FT_LOAD_RENDER) != 0) {
            fprintf(stderr, "Failed to load glyph '%c'\n", (char)ch);
            return -1;
        }

        FT_GlyphSlot g = face->glyph;
        int glyph_x = pen_x + g->bitmap_left;
        int glyph_y = baseline_y - g->bitmap_top;

        for (int row = 0; row < (int)g->bitmap.rows; row++) {
            for (int col = 0; col < (int)g->bitmap.width; col++) {
                unsigned char cov = g->bitmap.buffer[row * g->bitmap.pitch + col];
                if (!cov) {
                    continue;
                }

                int x = glyph_x + col;
                int y = glyph_y + row;

                if (outline_px > 0) {
                    int outline_alpha = cov * 70 / 255;
                    draw_thick_pixel(x, y, outline_px, outline_alpha, OL_R, OL_G, OL_B);
                }

                blend_pixel(x, y, cov, FG_R, FG_G, FG_B);

                for (int bx = 1; bx <= bold_px; bx++) {
                    blend_pixel(x + bx, y, cov, FG_R, FG_G, FG_B);
                }
            }
        }

        pen_x += g->advance.x >> 6;
        prev = glyph_index;
    }

    return 0;
}

int main(int argc, char **argv) {
    const char *text = (argc > 1) ? argv[1] : "VH-XVG";
    const char *outfile = (argc > 2) ? argv[2] : "callsign.jpg";
    int quality = (argc > 3) ? atoi(argv[3]) : 85;

    if (quality < 1) quality = 1;
    if (quality > 100) quality = 100;

    FT_Library ft = NULL;
    FT_Face face = NULL;

    if (FT_Init_FreeType(&ft) != 0) {
        fprintf(stderr, "FT_Init_FreeType failed\n");
        return 1;
    }

    if (FT_New_Memory_Face(ft,
                           cinzel_bold_ttf,
                           (FT_Long)cinzel_bold_ttf_len,
                           0,
                           &face) != 0) {
        fprintf(stderr, "FT_New_Memory_Face failed\n");
        FT_Done_FreeType(ft);
        return 1;
    }

    clear_image();

    if (render_text(face, text, 1, 0) != 0) {
        FT_Done_Face(face);
        FT_Done_FreeType(ft);
        return 1;
    }

    if (write_jpeg(outfile, quality) != 0) {
        FT_Done_Face(face);
        FT_Done_FreeType(ft);
        return 1;
    }

    FT_Done_Face(face);
    FT_Done_FreeType(ft);

    printf("Wrote %s for text \"%s\" at JPEG quality %d\n", outfile, text, quality);
    return 0;
}
