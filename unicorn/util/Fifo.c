#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>
#include <unistd.h>
#include <assert.h>
#include <pthread.h>

#include "Fifo.h"

#define MODULE_NAME     "Fifo"

typedef struct Fifo_Object {
    pthread_mutex_t mutex;
    int             numBufs;
    int16_t         flush;
    int             pipes[2];
} Fifo_Object;

const Fifo_Attrs Fifo_Attrs_DEFAULT = {
    0
};
/*
 * Fifo_create
 */
Fifo_Handle Fifo_create(Fifo_Attrs *attrs)
{
    Fifo_Handle hFifo;

    if (attrs == NULL) {
        printf("NULL attrs not supported\n");
        return NULL;
    }

    hFifo = calloc(1, sizeof(Fifo_Object));

    if (hFifo == NULL) {
        printf("Failed to allocate space for Fifo Object\n");
        return NULL;
    }

    if (pipe(hFifo->pipes)) {
        free(hFifo);
        return NULL;
    }

    pthread_mutex_init(&hFifo->mutex, NULL);

    return hFifo;
}
/*
 * Fifo_delete
 */
int Fifo_delete(Fifo_Handle hFifo)
{
    int ret = 0;

    if (hFifo) {
        if (close(hFifo->pipes[0])) {
            ret = -1;
        }

        if (close(hFifo->pipes[1])) {
            ret = -1;
        }

        pthread_mutex_destroy(&hFifo->mutex);

        free(hFifo);
    }

    return ret;
}
/*
 * Fifo_get
 */
int Fifo_get(Fifo_Handle hFifo, void **ptrPtr)
{
    int flush;
    int numBytes;

    assert(hFifo);
    assert(ptrPtr);

    pthread_mutex_lock(&hFifo->mutex);
    flush = hFifo->flush;
    pthread_mutex_unlock(&hFifo->mutex);

    if (flush) {
        return -1;
    }

    numBytes = read(hFifo->pipes[0], ptrPtr, sizeof(void *));

    if (numBytes != sizeof(void *)) {
        pthread_mutex_lock(&hFifo->mutex);
        flush = hFifo->flush;
        if (flush) {
            hFifo->flush = false;
        }
        pthread_mutex_unlock(&hFifo->mutex);

        if (flush) {
            return -1;
        }
        return -1;
    }

    pthread_mutex_lock(&hFifo->mutex);
    hFifo->numBufs--;
    pthread_mutex_unlock(&hFifo->mutex);

    return 0;
}
/*
 * Fifo_flush
 */
int Fifo_flush(Fifo_Handle hFifo)
{
    char ch = 0xff;

    assert(hFifo);

    pthread_mutex_lock(&hFifo->mutex);
    hFifo->flush = true;
    pthread_mutex_unlock(&hFifo->mutex);

    /* Make sure any Fifo_get() calls are unblocked */
    if (write(hFifo->pipes[1], &ch, 1) != 1) {
        return -1;
    }

    return 0;
}
/*
 * Fifo_put
 */
int Fifo_put(Fifo_Handle hFifo, void *ptr)
{
    assert(hFifo);
    //assert(ptr);

    pthread_mutex_lock(&hFifo->mutex);
    hFifo->numBufs++;
    pthread_mutex_unlock(&hFifo->mutex);

    if (write(hFifo->pipes[1], &ptr, sizeof(void *)) != sizeof(void *)) {
        return -1;
    }

    return 0;
}
/*
 * Fifo_getNumEntries
 */
int Fifo_getNumEntries(Fifo_Handle hFifo)
{
    int numEntries;

    assert(hFifo);

    pthread_mutex_lock(&hFifo->mutex);
    numEntries = hFifo->numBufs;
    pthread_mutex_unlock(&hFifo->mutex);

    return numEntries;
}
