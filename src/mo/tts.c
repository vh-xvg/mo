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

// tts.c
#include <pthread.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <sys/wait.h>
#include <errno.h>

#define TTS_QUEUE_MAX 16

struct tts_msg {
    char *text;
    struct tts_msg *next;
};

static pthread_mutex_t tts_lock = PTHREAD_MUTEX_INITIALIZER;
static pthread_cond_t  tts_cond = PTHREAD_COND_INITIALIZER;
static pthread_t       tts_thread;
static pthread_once_t  tts_once = PTHREAD_ONCE_INIT;

static struct tts_msg *tts_head, *tts_tail;
static unsigned tts_depth;
static int tts_stopping;
static int tts_thread_active = 0;

static void speak_english_blocking(const char *text)
{
    pid_t pid = fork();

    if (pid == 0) {
        execlp("espeak-ng", "espeak-ng",
               "-v", "en",
               "--",
               text,
               (char *)NULL);

        execlp("espeak", "espeak",
               "-v", "en",
               "--",
               text,
               (char *)NULL);

        _exit(127);
    }

    if (pid > 0) {
        int status;
        while (waitpid(pid, &status, 0) < 0 && errno == EINTR)
            ;
    }
}

static void *tts_worker(void *arg)
{
    (void)arg;

    for (;;) {
        pthread_mutex_lock(&tts_lock);

        while (!tts_head && !tts_stopping)
            pthread_cond_wait(&tts_cond, &tts_lock);

        if (tts_stopping && !tts_head) {
            pthread_mutex_unlock(&tts_lock);
            break;
        }

        struct tts_msg *msg = tts_head;
        tts_head = msg->next;
        if (!tts_head)
            tts_tail = NULL;
        tts_depth--;

        pthread_mutex_unlock(&tts_lock);

        speak_english_blocking(msg->text);

        free(msg->text);
        free(msg);
    }

    return NULL;
}

static void tts_start_once(void)
{
    pthread_create(&tts_thread, NULL, tts_worker, NULL);
    tts_thread_active = 1;
}

/*
 * Thread-safe public access function.
 *
 * Returns:
 *   0   queued successfully
 *  -1   invalid text, allocation failure, or queue full
 */
int tts_say(const char *text)
{
    if (!text || !*text)
        return -1;

    pthread_once(&tts_once, tts_start_once);

    struct tts_msg *msg = calloc(1, sizeof(*msg));
    if (!msg)
        return -1;

    msg->text = strdup(text);
    if (!msg->text) {
        free(msg);
        return -1;
    }

    pthread_mutex_lock(&tts_lock);

    if (tts_stopping || tts_depth >= TTS_QUEUE_MAX) {
        pthread_mutex_unlock(&tts_lock);
        free(msg->text);
        free(msg);
        return -1;
    }

    if (tts_tail)
        tts_tail->next = msg;
    else
        tts_head = msg;

    tts_tail = msg;
    tts_depth++;

    pthread_cond_signal(&tts_cond);
    pthread_mutex_unlock(&tts_lock);

    return 0;
}

void tts_shutdown(void)
{
    pthread_mutex_lock(&tts_lock);
    tts_stopping = 1;
    pthread_cond_signal(&tts_cond);
    pthread_mutex_unlock(&tts_lock);

    if (tts_thread_active)
      pthread_join(tts_thread, NULL);
}
