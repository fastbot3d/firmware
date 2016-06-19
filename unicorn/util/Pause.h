#ifndef PAUSE_H_
#define PAUSE_H_
/**
 * @brief       Handle through which to reference a Pause Object.
 */
typedef struct Pause_Object *Pause_Handle;

/**
 * @brief       Attributes used to create a Pause.
 * @see         Pause_Attrs_DEFAULT.
 */
typedef struct Pause_Attrs {
    /** @brief Currently no attributes for this module, but this may change. */
    int dummy;
} Pause_Attrs;

/**
 * @brief       Default attributes for a Pause.
 */
extern const Pause_Attrs Pause_Attrs_DEFAULT;

#if defined (__cplusplus)
extern "C" {
#endif

/**
 * @brief       Creates a Pause object.
 *
 * @param[in]   attrs       #Pause_Attrs to use for creating the Pause Object.
 *
 * @retval      Handle for use in subsequent operations (see #Pause_Handle).
 * @retval      NULL for failure.
 */
extern Pause_Handle Pause_create(Pause_Attrs *attrs);

/**
 * @brief       Called to see if processing is supposed to pause. If so, block
 *              the execution of the thread. Otherwise continue.
 *
 * @param[in]   hPause      #Pause_Handle to test.
 *
 * @remarks     #Pause_create must be called before this function.
 */
extern void Pause_test(Pause_Handle hPause);

/**
 * @brief       Called to set processing threads to pause.
 *
 * @param[in]   hPause      #Pause_Handle to pause.
 *
 * @remarks     #Pause_create must be called before this function.
 */
extern void Pause_on(Pause_Handle hPause);

/**
 * @brief       Called to release processing threads currently paused.
 *
 * @param[in]   hPause      #Pause_Handle to release.
 *
 * @remarks     #Pause_create must be called before this function.
 */
extern void Pause_off(Pause_Handle hPause);

/**
 * @brief       Deletes a Pause object.
 *
 * @param[in]   hPause      #Pause_Handle of the Paise object to delete.
 *
 * @retval      Dmai_EOK for success.
 * @retval      "Negative value" for failure, see Dmai.h.
 *
 * @remarks     #Pause_create must be called before this function.
 */
extern int Pause_delete(Pause_Handle hPause);

#if defined (__cplusplus)
}
#endif

#endif
