#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>
#include <unistd.h>
#include <assert.h>
#include <pthread.h>

#include "Pause.h"

#define MODULE_NAME     "Pause"

typedef struct Pause_Object {
    int             pause;
    pthread_mutex_t mutex;
    pthread_cond_t  cond;
} Pause_Object;

const Pause_Attrs Pause_Attrs_DEFAULT = {
    0
};
/*
 * Pause_create
 */
Pause_Handle Pause_create(Pause_Attrs *attrs)
{
    Pause_Handle hPause;

    hPause = calloc(1, sizeof(Pause_Object));

    if (hPause == NULL) {
        printf("Failed to allocate space for Pause Object\n");
        return NULL;
    }

    pthread_mutex_init(&hPause->mutex, NULL);
    pthread_cond_init(&hPause->cond, NULL);

    hPause->pause = 0;

    return hPause;
}
/*
 * Pause_delete
 */
int Pause_delete(Pause_Handle hPause)
{
    if (hPause) {
        pthread_mutex_destroy(&hPause->mutex);
        pthread_cond_destroy(&hPause->cond);
        free(hPause);
    }

    return 0;
}
/*
 * Pause_test
 */
void Pause_test(Pause_Handle hPause)
{
    assert(hPause);

    pthread_mutex_lock(&hPause->mutex);
    if (hPause->pause == 1) {
        pthread_cond_wait(&hPause->cond, &hPause->mutex);
    }
    pthread_mutex_unlock(&hPause->mutex);
}
/*
 * Pause_on
 */
void Pause_on(Pause_Handle hPause)
{
    assert(hPause);

    pthread_mutex_lock(&hPause->mutex);
    hPause->pause = 1;
    pthread_mutex_unlock(&hPause->mutex);
}
/*
 * Pause_off
 */
void Pause_off(Pause_Handle hPause)
{
    assert(hPause);

    pthread_mutex_lock(&hPause->mutex);
    if (hPause->pause == 1) {
        hPause->pause = 0;
        pthread_cond_broadcast(&hPause->cond);
    }
    pthread_mutex_unlock(&hPause->mutex);
}
