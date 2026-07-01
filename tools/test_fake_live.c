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
#include <string.h>

#include "vibsense_fake_window.h"
#include "vibsense_live.h"

static int analyze_one(const vibsense_window_t *win,
                       const vibsense_live_config_t *cfg,
                       vibsense_live_metrics_t *m)
{
    memset(m, 0, sizeof(*m));
    int rc = vibsense_live_analyze_window(win, cfg, m);
    if (rc != VIBSENSE_LIVE_OK)
    {
        fprintf(stderr, "analyze failed rc=%d\n", rc);
        return 1;
    }
    return 0;
}

static void print_metric(int i, const char *label, const vibsense_live_metrics_t *m)
{
    printf("%02d %-4s src=%-4s rpm=%7.1f ips=%0.3f phase=%6.1f "
           "dot=(%+.2f,%+.2f) q=0x%08x samples=%u revs=%u\n",
           i,
           label,
           vibsense_fake_source_name(m->accel_source),
           m->rpm,
           m->ips_xy,
           m->phase_deg,
           m->dot_x_norm,
           m->dot_y_norm,
           m->quality_flags,
           m->sample_count,
           m->revolutions);
}

int main(void)
{
    vibsense_live_config_t cfg;
    vibsense_live_default_config(&cfg);
    cfg.display_full_scale_ips = 0.25;

    vibsense_fake_reset();

    puts("Single IIS windows:");
    for (int i = 0; i < 4; i++)
    {
        vibsense_window_t win;
        vibsense_live_metrics_t m;

        if (vibsense_fake_fill_window_source(&win, VIB_ACCEL_SOURCE_IIS3DWBG1) != 0)
            return 1;
        if (analyze_one(&win, &cfg, &m))
            return 1;

        print_metric(i, "IIS", &m);
    }

    puts("");
    puts("Dual IIS + ADXL windows:");
    for (int i = 0; i < 12; i++)
    {
        vibsense_window_t windows[2];
        vibsense_live_metrics_t metrics[2];

        if (vibsense_fake_fill_both_windows(windows) != 0)
        {
            fprintf(stderr, "fake fill both failed\n");
            return 1;
        }

        if (analyze_one(&windows[0], &cfg, &metrics[0]))
            return 1;
        if (analyze_one(&windows[1], &cfg, &metrics[1]))
            return 1;

        printf("case %-28s\n", vibsense_fake_scenario_name((uint32_t)i));
        print_metric(i, "IIS",  &metrics[0]);
        print_metric(i, "ADXL", &metrics[1]);
    }

    return 0;
}
