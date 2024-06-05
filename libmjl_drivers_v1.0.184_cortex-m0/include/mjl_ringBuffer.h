/***************************************************************************
*                                Majestic Labs © 2023
* File: mjl_ringBuffer.h
* Workspace: MJL Driver Library 
* Version: v1.0.0
* Author: C. Cheney
*
* Brief: implement a ring buffer 
*
* 2023.06.28  - Document Created
********************************************************************************/
/* Header Guard */
#ifndef MJL_RING_BUFFER_H
  #define MJL_RING_BUFFER_H
  /***************************************
  * Included files
  ***************************************/
  #include <stdbool.h>
  #include <mjl_errors.h>
  /***************************************
  * Macro Definitions
  ***************************************/

  /***************************************
  * Enumerated types
  ***************************************/

  /***************************************
  * Structures 
  ***************************************/

  /* Configuration Structure */
  typedef struct {
    uint32_t *buffer;
    uint16_t size;
    bool overWrite;
  } mjl_ring_cfg_s;

  /* Serial State Object   */
  typedef struct {
    uint32_t *buffer;
    uint16_t size;
    bool overWrite;
    uint16_t head;
    uint16_t tail;
    uint16_t count;
    bool _init;
  } mjl_ring_s;

  /* Default config struct */
  extern const mjl_ring_cfg_s mjl_ring_cfg_default;
  /***************************************
  * Function declarations 
  ***************************************/
  /* State Operations */
  uint32_t mjl_ringBuffer_init(mjl_ring_s *const state, mjl_ring_cfg_s *const cfg);
  uint32_t mjl_ringBuffer_enqueue(mjl_ring_s *const state, uint32_t in);
  uint32_t mjl_ringBuffer_dequeue(mjl_ring_s *const state, uint32_t *out);

  bool mjl_ringBuffer_isBufferFull(mjl_ring_s *const state);
  bool mjl_ringBuffer_isBufferEmpty(mjl_ring_s *const state);


#endif /* MJL_RING_BUFFER_H */
/* [] END OF FILE */
