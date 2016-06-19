#ifndef Fifo_h_
#define Fifo_h_
/**
 * @brief       Handle through which to reference a Fifo.
 */
typedef struct Fifo_Object *Fifo_Handle;

/**
 * @brief       Attributes used to create a Fifo.
 * @see         Fifo_Attrs_DEFAULT.
 */
typedef struct Fifo_Attrs {
    /** 
     * @brief      Maximum elements that can be put on the Fifo at once
     * @remarks    For Bios only, Linux ignores this attribute
     */     
    int maxElems;
} Fifo_Attrs;

/**
 * @brief       Default attributes for a Fifo.
 * @code
 * numElems     = 20
 * @endcode
 */
extern const Fifo_Attrs Fifo_Attrs_DEFAULT;

#if defined (__cplusplus)
extern "C" {
#endif

/**
 * @brief       Creates a fifo.
 *
 * @param[in]   attrs       #Fifo_Attrs to use for creating the Fifo.
 *
 * @retval      Handle for use in subsequent operations (see #Fifo_Handle).
 * @retval      NULL for failure.
 */
extern Fifo_Handle Fifo_create(Fifo_Attrs *attrs);

/**
 * @brief       Blocking call to receive a buffer pointer from a fifo.
 *
 * @param[in]   hFifo       #Fifo_Handle from which to receive a buffer.
 * @param[out]  ptrPtr      A pointer to the pointer to be set.
 *
 * @retval      Dmai_EOK if a buffer was successfully received.
 * @retval      Dmai_EFLUSH if the fifo was flushed.
 * @retval      "Negative value" for failure, see Dmai.h.
 *
 * @remarks     #Fifo_create must be called before this function.
 */
extern int Fifo_get(Fifo_Handle hFifo, void **ptrPtr);

/**
 * @brief       Flushes a fifo. The other end will unblock and return the
 *              (non-negative) #Dmai_EFLUSH error code.
 *
 * @param[in]   hFifo       #Fifo_Handle which to flush.
 *
 * @retval      Dmai_EOK for success.
 * @retval      "Negative value" for failure, see Dmai.h.
 *
 * @remarks     #Fifo_create must be called before this function.
 */
extern int Fifo_flush(Fifo_Handle hFifo);

/**
 * @brief       Put a buffer pointer on the fifo.
 *
 * @param[in]   hFifo       #Fifo_Handle to which to send a buffer.
 * @param[in]   ptr         The pointer to put to the fifo.
 *
 * @retval      Dmai_EOK for success.
 * @retval      "Negative value" for failure, see Dmai.h.
 *
 * @remarks     #Fifo_create must be called before this function.
 */
extern int Fifo_put(Fifo_Handle hFifo, void *ptr);

/**
 * @brief       Determine number of entries (pointers) currently in fifo.
 *
 * @param[in]   hFifo       #Fifo_Handle which to investigate.
 *
 * @retval      Number of entries in the fifo on success.
 * @retval      "Negative value" for failure, see Dmai.h.
 *
 * @remarks     #Fifo_create must be called before this function.
 */
extern int Fifo_getNumEntries(Fifo_Handle hFifo);

/**
 * @brief       Deletes a previously created fifo.
 *
 * @param[in]   hFifo       #Fifo_Handle for the fifo to delete.
 *
 * @retval      Dmai_EOK for success.
 * @retval      "Negative value" for failure, see Dmai.h.
 *
 * @remarks     #Fifo_create must be called before this function.
 */
extern int Fifo_delete(Fifo_Handle hFifo);

#if defined (__cplusplus)
}
#endif
#endif
