/*
 * Copyright 2005-2006 Freescale Semiconductor, Inc. All Rights Reserved.
 */

 /*
  * The code contained herein is licensed under the GNU Lesser General
  * Public License.  You may obtain a copy of the GNU Lesser General
  * Public License Version 2.1 or later at the following locations:
  *
  * http://www.opensource.org/licenses/lgpl-license.html
  * http://www.gnu.org/copyleft/lgpl.html
  */

#ifndef SAHARA2_API_H
#define SAHARA2_API_H

#ifdef DIAG_SECURITY_FUNC
#include <diagnostic.h>
#endif /* DIAG_SECURITY_FUNC */

/* This is a Linux flag... ? */
#ifndef __KERNEL__
#include <inttypes.h>
#include <stdlib.h>
#include <memory.h>
#else
#include "portable_os.h"
#endif


/* This definition may need a new name, and needs to go somewhere which
 * can determine platform, kernel vs. user, os, etc.
 */
#define copy_bytes(out, in, len) memcpy(out, in, len)


/* Does this belong here? */
#ifndef SAHARA_DEVICE
#define SAHARA_DEVICE "/dev/sahara"
#endif

/*!
 * @defgroup MXCSAHARA2 Sahara2 Driver
 *
 * @ingroup MXCSECDRVRS
 */

/*!
*******************************************************************************
* @file sahara.h
*
* @brief File which implements the FSL_SHW API when used on Sahara
*
* @ingroup MXCSAHARA2
*
******************************************************************************/
/*!
 * @name Link Flags
 * Flags to show information about link data and link segments
 */
/*! @{ */

/*!
*******************************************************************************
* This flag indicates that the data in a link is owned by the security
* function component and this memory will be freed by the security function
* component. To be used as part of the flag field of the sah_Link structure.
******************************************************************************/
#define SAH_OWNS_LINK_DATA          0x01

/*!
*******************************************************************************
* The data in a link is not owned by the security function component and
* therefore it will not attempt to free this memory. To be used as part of the
* flag field of the sah_Link structure.
******************************************************************************/
#define SAH_USES_LINK_DATA          0x02

/*!
*******************************************************************************
* The data in this link will change when the descriptor gets executed.
******************************************************************************/
#define SAH_OUTPUT_LINK             0x04

/*!
*******************************************************************************
* The ptr and length in this link are really 'established key' info.  They
* are to be converted to ptr/length before putting on request queue.
******************************************************************************/
#define SAH_KEY_IS_HIDDEN           0x08

/*!
*******************************************************************************
* The link structure has been appended to the previous one by the driver.  It
* needs to be removed before leaving the driver (and returning to API).
******************************************************************************/
#define SAH_REWORKED_LINK           0x10

/*!
*******************************************************************************
* The length and data fields of this link contain the slot and user id
* used to access the SCC stored key
******************************************************************************/
#define SAH_STORED_KEY_INFO         0x20


/*!
*******************************************************************************
* The Data field points to a physical address, and does not need to be
* processed by the driver.  Honored only in Kernel API.
******************************************************************************/
#define SAH_PREPHYS_DATA            0x40


/*!
*******************************************************************************
* The link was inserted during the Physicalise procedure.  It is tagged so
* it can be removed during DePhysicalise, thereby returning to the caller an
* intact chain.
******************************************************************************/
#define SAH_LINK_INSERTED_LINK 0x80
/*! @} */


/*!
*******************************************************************************
* sah_Link_Flags
*
* Type to be used for flags associated with a Link in security function.
* These flags are used internally by the security function component only.
*
* Values defined at "Link Flags"
*
* @brief typedef for flags field of sah_Link
******************************************************************************/
typedef uint8_t sah_Link_Flags;



/*
*******************************************************************************
* Security Parameters Related Structures
*
* All of structures associated with API parameters
*
******************************************************************************/

/*
*******************************************************************************
* Common Types
*
* All of structures used across several classes of crytography
******************************************************************************/

/*!
*******************************************************************************
* @brief Indefinite precision integer used for security operations on SAHARA
* accelerator. The data will always be in little Endian format.
******************************************************************************/
typedef uint8_t * sah_Int;

/*!
*******************************************************************************
* @brief Byte array used for block cipher and hash digest/MAC operations on
* SAHARA accelerator. The Endian format will be as specified by the function
* using the sah_Oct_Str.
******************************************************************************/
typedef uint8_t * sah_Oct_Str;


/*!
 * A queue of descriptor heads -- used to hold requests waiting for user to
 * pick up the results. */
typedef struct sah_Queue
{
    int count;                  /*!< # entries in queue  */
    struct sah_Head_Desc  *head; /*!< first entry in queue  */
    struct sah_Head_Desc  *tail; /*!< last entry in queue   */
} sah_Queue;


/******************************************************************************
 * Enumerations
 *****************************************************************************/
/*!
 * Flags for the state of the User Context Object (#fsl_shw_uco_t).
 */
typedef enum fsl_shw_user_ctx_flags
{
    FSL_UCO_BLOCKING_MODE=1,/*!< API will block the caller until operation
                               completes.  The result will be available in the
                               return code.  If this is not set, user will have
                               to get results using #fsl_shw_get_results(). */
    FSL_UCO_CALLBACK_MODE=2,/*!< User wants callback (at the function specified
                               with #fsl_shw_uco_set_callback()) when the
                               operation completes.  This flag is valid only if
                               #FSL_UCO_BLOCKING_MODE is not set. */
    FSL_UCO_SAVE_DESC_CHAIN=4,  /*!< Do not free descriptor chain after driver
                                   (adaptor) finishes */
    FSL_UCO_CALLBACK_SETUP_COMPLETE = 8, /*!< User has made at least one
                                           request with callbacks requested, so
                                           API is ready to handle others. */

    FSL_UCO_CHAIN_PREPHYSICALIZED = 16 /*!< (virtual) pointer to descriptor
                                    chain is completely linked with physical
                                    (DMA) addresses, ready for the hardware.
                                    This flag should not be used by FSL SHW API
                                    programs. */
} fsl_shw_user_ctx_flags_t;


/*!
 * Return code for FSL_SHW library.
 *
 * These codes may be returned from a function call.  In non-blocking mode,
 * they will appear as the status in a Result Object.
 */
typedef enum fsl_shw_return_t
{
    FSL_RETURN_OK_S = 0,        /*!< No error.  As a function return code in
                                   Non-blocking mode, this may simply mean that
                                   the operation was accepted for eventual
                                   execution. */
    FSL_RETURN_ERROR_S,         /*!< Failure for non-specific reason. */
    FSL_RETURN_NO_RESOURCE_S,   /*!< Operation failed because some resource was
                                   not able to be allocated. */
    FSL_RETURN_BAD_ALGORITHM_S, /*!< Crypto algorithm unrecognized or
                                   improper. */
    FSL_RETURN_BAD_MODE_S,      /*!< Crypto mode unrecognized or improper. */
    FSL_RETURN_BAD_FLAG_S,      /*!< Flag setting unrecognized or
                                   inconsistent. */
    FSL_RETURN_BAD_KEY_LENGTH_S, /*!< Improper or unsupported key length for
                                    algorithm. */
    FSL_RETURN_BAD_KEY_PARITY_S, /*!< Improper parity in a (DES, TDES) key. */
    FSL_RETURN_BAD_DATA_LENGTH_S, /*!< Improper or unsupported data length for
                                    algorithm or internal buffer. */
    FSL_RETURN_AUTH_FAILED_S,    /*!< Authentication failed in
                                    authenticate-decrypt operation. */
    FSL_RETURN_MEMORY_ERROR_S,  /*!< A memory error occurred. */
    FSL_RETURN_INTERNAL_ERROR_S /*!< An error internal to the hardware
                                   occurred. */
} fsl_shw_return_t;


/*!
 * Algorithm Identifier.
 *
 * Selection of algorithm will determine how large the block size of the
 * algorithm is.   Context size is the same length unless otherwise specified.
 * Selection of algorithm also affects the allowable key length.
 */
typedef enum fsl_shw_key_alg_t
{
    FSL_KEY_ALG_HMAC,           /*!< Key will be used to perform an HMAC.  Key
                                   size is 1 to 64 octets.  Block size is 64
                                   octets. */
    FSL_KEY_ALG_AES,            /*!< Advanced Encryption Standard (Rijndael).
                                   Block size is 16 octets.  Key size is 64
                                   octets.  (The single choice of key size is a
                                   Sahara platform limitation.) */
    FSL_KEY_ALG_DES,            /*!< Data Encryption Standard.  Block size is
                                   8 octets.  Key size is 8 octets. */
    FSL_KEY_ALG_TDES,           /*!< 2- or 3-key Triple DES.  Block size is 8
                                   octets.  Key size is 16 octets for 2-key
                                   Triple DES, and 24 octets for 3-key. */
    FSL_KEY_ALG_ARC4            /*!< ARC4.  No block size.  Context size is 259
                                   octets.  Allowed key size is 1-16 octets.
                                   (The choices for key size are a Sahara
                                   platform limitation.) */
} fsl_shw_key_alg_t;


/*!
 * Mode selector for Symmetric Ciphers.
 *
 * The selection of mode determines how a cryptographic algorithm will be
 * used to process the plaintext or ciphertext.
 *
 * For all modes which are run block-by-block (that is, all but
 * #FSL_SYM_MODE_STREAM), any partial operations must be performed on a text
 * length which is multiple of the block size.  Except for #FSL_SYM_MODE_CTR,
 * these block-by-block algorithms must also be passed a total number of octets
 * which is a multiple of the block size.
 *
 * In modes which require that the total number of octets of data be a multiple
 * of the block size (#FSL_SYM_MODE_ECB and #FSL_SYM_MODE_CBC), and the user
 * has a total number of octets which are not a multiple of the block size, the
 * user must perform any necessary padding to get to the correct data length.
 */
typedef enum fsl_shw_sym_mode_t
{
    /*!
     * Stream.  There is no associated block size.  Any request to process data
     * may be of any length.  This mode is only for ARC4 operations, and is
     * also the only mode used for ARC4.
     */
    FSL_SYM_MODE_STREAM,

    /*!
     * Electronic Codebook.  Each block of data is encrypted/decrypted.  The
     * length of the data stream must be a multiple of the block size.  This
     * mode may be used for DES, 3DES, and AES.  The block size is determined
     * by the algorithm.
     */
    FSL_SYM_MODE_ECB,
    /*!
     * Cipher-Block Chaining.  Each block of data is encrypted/decrypted and
     * then "chained" with the previous block by an XOR function.  Requires
     * context to start the XOR (previous block).  This mode may be used for
     * DES, 3DES, and AES.  The block size is determined by the algorithm.
     */
    FSL_SYM_MODE_CBC,
    /*!
     * Counter.  The counter is encrypted, then XORed with a block of data.
     * The counter is then incremented (using modulus arithmetic) for the next
     * block. The final operation may be non-multiple of block size.  This mode
     * may be used for AES.  The block size is determined by the algorithm.
     */
    FSL_SYM_MODE_CTR,
} fsl_shw_sym_mode_t;


/*!
 * Algorithm selector for Cryptographic Hash functions.
 *
 * Selection of algorithm determines how large the context and digest will be.
 * Context is the same size as the digest (resulting hash), unless otherwise
 * specified.
 */
typedef enum fsl_shw_hash_alg
{
    FSL_HASH_ALG_MD5,           /*!< MD5 algorithm.  Digest is 16 octets. */
    FSL_HASH_ALG_SHA1,          /*!< SHA-1 (aka SHA or SHA-160) algorithm.
                                   Digest is 20 octets. */
    FSL_HASH_ALG_SHA224,        /*!< SHA-224 algorithm.  Digest is 28 octets,
                                   though context is 32 octets. */
    FSL_HASH_ALG_SHA256         /*!< SHA-256 algorithm.  Digest is 32
                                   octets. */
} fsl_shw_hash_alg_t;


/*!
 * The type of Authentication-Cipher function which will be performed.
 */
typedef enum fsl_shw_acc_mode_t
{
    /*!
     * CBC-MAC for Counter.  Requires context and modulus.  Final operation may
     * be non-multiple of block size.  This mode may be used for AES.
     */
    FSL_ACC_MODE_CCM,
    /*!
     * SSL mode.  Not supported.  Combines HMAC and encrypt (or decrypt).
     * Needs one key object for encryption, another for the HMAC.  The usual
     * hashing and symmetric encryption algorithms are supported.
     */
    FSL_ACC_MODE_SSL
} fsl_shw_acc_mode_t;


/* REQ-S2LRD-PINTFC-COA-HCO-001 */
/*!
 * Flags which control a Hash operation.
 */
typedef enum fsl_shw_hash_ctx_flags
{
    FSL_HASH_FLAGS_INIT = 1,        /*!< Context is empty.  Hash is started
                                       from scratch, with a message-processed
                                       count of zero. */
    FSL_HASH_FLAGS_SAVE = 2,        /*!< Retrieve context from hardware after
                                       hashing.  If used with the
                                       #FSL_HASH_FLAGS_FINALIZE flag, the final
                                       digest value will be saved in the
                                       object. */
    FSL_HASH_FLAGS_LOAD = 4,        /*!< Place context into hardware before
                                       hashing. */
    FSL_HASH_FLAGS_FINALIZE = 8,    /*!< PAD message and perform final digest
                                       operation.  If user message is
                                       pre-padded, this flag should not be
                                       used. */
} fsl_shw_hash_ctx_flags_t;


/*!
 * Flags which control an HMAC operation.
 *
 * These may be combined by ORing them together.  See #fsl_shw_hmco_set_flags()
 * and #fsl_shw_hmco_clear_flags().
 */
typedef enum fsl_shw_hmac_ctx_flags_t
{
    FSL_HMAC_FLAGS_INIT = 1,        /*!< Message context is empty.  HMAC is
                                       started from scratch (with key) or from
                                       precompute of inner hash, depending on
                                       whether
                                       #FSL_HMAC_FLAGS_PRECOMPUTES_PRESENT is
                                       set. */
    FSL_HMAC_FLAGS_SAVE = 2,        /*!< Retrieve ongoing context from hardware
                                       after hashing.  If used with the
                                       #FSL_HMAC_FLAGS_FINALIZE flag, the final
                                       digest value (HMAC) will be saved in the
                                       object. */
    FSL_HMAC_FLAGS_LOAD = 4,        /*!< Place ongoing context into hardware
                                       before hashing. */
    FSL_HMAC_FLAGS_FINALIZE = 8,    /*!< PAD message and perform final HMAC
                                       operations of inner and outer hashes. */
    FSL_HMAC_FLAGS_PRECOMPUTES_PRESENT = 16 /*!< This means that the context
                                        contains precomputed inner and outer
                                        hash values. */
} fsl_shw_hmac_ctx_flags_t;



/*!
 * Flags to control use of the #fsl_shw_scco_t.
 *
 * These may be ORed together to get the desired effect.
 * See #fsl_shw_scco_set_flags() and #fsl_shw_scco_clear_flags()
 */
typedef enum fsl_shw_sym_ctx_flags_t
{
    /*!
     * Context is empty.  In ARC4, this means that the S-Box needs to be
     * generated from the key.  In #FSL_SYM_MODE_CBC mode, this allows an IV of
     * zero to be specified.  In #FSL_SYM_MODE_CTR mode, it means that an
     * initial CTR value of zero is desired.
     */
    FSL_SYM_CTX_INIT = 1,
    /*!
     * Load context from object into hardware before running cipher.  In
     * #FSL_SYM_MODE_CTR mode, this would refer to the Counter Value.
     */
    FSL_SYM_CTX_LOAD = 2,
    /*!
     * Save context from hardware into object after running cipher.  In
     * #FSL_SYM_MODE_CTR mode, this would refer to the Counter Value.
     */
    FSL_SYM_CTX_SAVE = 4,
    /*!
     * Context (SBox) is to be unwrapped and wrapped on each use.
     * This flag is unsupported.
     * */
    FSL_SYM_CTX_PROTECT = 8,
} fsl_shw_sym_ctx_flags_t;


/*!
 * Flags which describe the state of the #fsl_shw_sko_t.
 *
 * These may be ORed together to get the desired effect.
 * See #fsl_shw_sko_set_flags() and #fsl_shw_sko_clear_flags()
 */
typedef enum fsl_shw_key_flags_t
{
    FSL_SKO_KEY_IGNORE_PARITY = 1, /*!< If algorithm is DES or 3DES, do not
                                      validate the key parity bits. */
    FSL_SKO_KEY_PRESENT = 2,       /*!< Clear key is present in the object. */
    FSL_SKO_KEY_ESTABLISHED = 4,   /*!< Key has been established for use.  This
                                       feature is not available for all
                                       platforms, nor for all algorithms and
                                       modes.*/
} fsl_shw_key_flags_t;


/*!
 * Type of value which is associated with an established key.
 */
typedef uint64_t key_userid_t;


/*!
 * Flags which describe the state of the #fsl_shw_acco_t.
 *
 * The @a FSL_ACCO_CTX_INIT and @a FSL_ACCO_CTX_FINALIZE flags, when used
 * together, provide for a one-shot operation.
 */
typedef enum fsl_shw_auth_ctx_flags_t
{
    FSL_ACCO_CTX_INIT = 1,      /*!< Initialize Context(s) */
    FSL_ACCO_CTX_LOAD = 2,      /*!< Load intermediate context(s).
                                   This flag is unsupported. */
    FSL_ACCO_CTX_SAVE = 4,      /*!< Save intermediate context(s).
                                   This flag is unsupported. */
    FSL_ACCO_CTX_FINALIZE = 8,  /*!< Create MAC during this operation. */
    FSL_ACCO_NIST_CCM = 0x10,   /*!< Formatting of CCM input data is
                                    performed by calls to
                                    #fsl_shw_ccm_nist_format_ctr_and_iv() and
                                    #fsl_shw_ccm_nist_update_ctr_and_iv().  */
}  fsl_shw_auth_ctx_flags_t;


/*!
 * The operation which controls the behavior of #fsl_shw_establish_key().
 *
 * These values are passed to #fsl_shw_establish_key().
 */
typedef enum fsl_shw_key_wrap_t
{
    FSL_KEY_WRAP_CREATE,        /*!< Generate a key from random values. */
    FSL_KEY_WRAP_ACCEPT,        /*!< Use the provided clear key. */
    FSL_KEY_WRAP_UNWRAP         /*!< Unwrap a previously wrapped key. */
} fsl_shw_key_wrap_t;


/*!
 *  Modulus Selector for CTR modes.
 *
 * The incrementing of the Counter value may be modified by a modulus.  If no
 * modulus is needed or desired for AES, use #FSL_CTR_MOD_128.
 */
typedef enum fsl_shw_ctr_mod
{
    FSL_CTR_MOD_8,              /*!< Run counter with modulus of 2^8. */
    FSL_CTR_MOD_16,             /*!< Run counter with modulus of 2^16. */
    FSL_CTR_MOD_24,             /*!< Run counter with modulus of 2^24. */
    FSL_CTR_MOD_32,             /*!< Run counter with modulus of 2^32. */
    FSL_CTR_MOD_40,             /*!< Run counter with modulus of 2^40. */
    FSL_CTR_MOD_48,             /*!< Run counter with modulus of 2^48. */
    FSL_CTR_MOD_56,             /*!< Run counter with modulus of 2^56. */
    FSL_CTR_MOD_64,             /*!< Run counter with modulus of 2^64. */
    FSL_CTR_MOD_72,             /*!< Run counter with modulus of 2^72. */
    FSL_CTR_MOD_80,             /*!< Run counter with modulus of 2^80. */
    FSL_CTR_MOD_88,             /*!< Run counter with modulus of 2^88. */
    FSL_CTR_MOD_96,             /*!< Run counter with modulus of 2^96. */
    FSL_CTR_MOD_104,            /*!< Run counter with modulus of 2^104. */
    FSL_CTR_MOD_112,            /*!< Run counter with modulus of 2^112. */
    FSL_CTR_MOD_120,            /*!< Run counter with modulus of 2^120. */
    FSL_CTR_MOD_128             /*!< Run counter with modulus of 2^128. */
} fsl_shw_ctr_mod_t;



/******************************************************************************
 * Data Structures
 *****************************************************************************/

/*!
 *
 * @brief Structure type for descriptors
 *
 * The first five fields are passed to the hardware.
 *
 *****************************************************************************/
typedef struct sah_Desc
{
    uint32_t          header;   /*!< descriptor header value */
    uint32_t          len1;     /*!< number of data bytes in 'ptr1' buffer */
    void *            ptr1;     /*!< pointer to first sah_Link structure */
    uint32_t          len2;     /*!< number of data bytes in 'ptr2' buffer */
    void *            ptr2;     /*!< pointer to second sah_Link structure */
    struct sah_Desc * next;     /*!< pointer to next descriptor */
#ifdef __KERNEL__ /* This needs a better test */
    /* These two must be last.  See sah_Copy_Descriptors */
    struct sah_Desc * virt_addr; /*!< Virtual (kernel) address of this
                                    descriptor. */
    dma_addr_t        dma_addr; /*!< Physical (bus) address of this
                                   descriptor.  */
    void *            original_ptr1; /*!< user's pointer to ptr1 */
    void *            original_ptr2; /*!< user's pointer to ptr2 */
    struct sah_Desc * original_next; /*!< user's pointer to next */
#endif
} sah_Desc;


/*!
*******************************************************************************
* @brief The first descriptor in a chain
******************************************************************************/
typedef struct sah_Head_Desc
{
    sah_Desc              desc; /*!< whole struct - must be first */
    struct fsl_shw_uco_t* user_info; /*!< where result pool lives */
    uint32_t              user_ref; /*!< at time of request */
    uint32_t              uco_flags; /*!< at time of request */
    uint32_t              status; /*!<  Status of queue entry */
    uint32_t              error_status; /*!< If error, register from Sahara */
    uint32_t              fault_address; /*!< If error, register from Sahara */
    uint32_t              current_dar; /*!< If error, register from Sahara */
    fsl_shw_return_t      result; /*!< Result of running descriptor  */
    struct sah_Head_Desc* next; /*!< Next in queue  */
    struct sah_Head_Desc* prev; /*!< previous in queue  */
    struct sah_Head_Desc* user_desc; /*!< For API async get_results */
    void*                 out1_ptr; /*!< For async post-processing  */
    void*                 out2_ptr; /*!< For async post-processing  */
    uint32_t              out_len; /*!< For async post-processing  */
} sah_Head_Desc;

/*!
 * @brief Structure type for links
 *
 * The first three fields are used by hardware.
 *****************************************************************************/
typedef struct sah_Link
{
    size_t            len;      /*!< len of 'data' buffer in bytes */
    uint8_t *         data;     /*!< buffer to store data */
    struct sah_Link * next;     /*!< pointer to the next sah_Link storing
                                 * data */
    sah_Link_Flags    flags;    /*!< indicates the component that created the
                                 * data buffer. Security Function internal
                                 * information */
    key_userid_t      ownerid;      /*!< Auth code for established key */
    uint32_t          slot;     /*!< Location of the the established key */
#ifdef __KERNEL__ /* This needs a better test */
    /* These two elements must be last.  See sah_Copy_Links() */
    struct sah_Link * virt_addr;
    dma_addr_t        dma_addr;
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,0))
    struct page  *    vm_info;
#endif
    uint8_t *         original_data; /*!< user's version of data pointer */
    struct sah_Link * original_next; /*!< user's version of next pointer */
#ifdef SAH_COPY_DATA
    uint8_t *         copy_data; /*!< Virtual address of acquired buffer */
#endif
#endif /* kernel-only */
} sah_Link;



/*!
 * Initialization Object
 */
typedef struct fsl_sho_ibo
{
} fsl_sho_ibo_t;


/* Imported from Sahara1 driver -- is it needed forever? */
/*!
*******************************************************************************
* FIELDS
*
*   void * ref - parameter to be passed into the memory function calls
*
*   void * (*malloc)(void *ref, size_t n) - pointer to user's malloc function
*
*   void   (*free)(void *ref, void *ptr)  - pointer to user's free function
*
*   void * (*memcpy)(void *ref, void *dest, const void *src, size_t n) -
*                                         pointer to user's memcpy function
*
*   void * (*memset)(void *ref, void *ptr, int ch, size_t n) - pointer to
*                                         user's memset function
*
* @brief Structure for API memory utilities
******************************************************************************/
typedef struct sah_Mem_Util
{
    /*! Who knows.  Vestigial. */
    void * mu_ref;
    /*! Acquire buffer of size n bytes */
    void * (*mu_malloc)(void *ref, size_t n);
    /*! Acquire a sah_Head_Desc */
    sah_Head_Desc * (*mu_alloc_head_desc)(void *ref);
    /*! Acquire a sah_Desc */
    sah_Desc * (*mu_alloc_desc)(void *ref);
    /*! Acquire a sah_Link */
    sah_Link * (*mu_alloc_link)(void *ref);
    /*! Free buffer at ptr */
    void   (*mu_free)(void *ref, void *ptr);
    /*! Free sah_Head_Desc at ptr */
    void   (*mu_free_head_desc)(void *ref, sah_Head_Desc *ptr);
    /*! Free sah_Desc at ptr */
    void   (*mu_free_desc)(void *ref, sah_Desc *ptr);
    /*! Free sah_Link at ptr */
    void   (*mu_free_link)(void *ref, sah_Link *ptr);
    /*! Funciton which will copy n bytes from src to dest  */
    void * (*mu_memcpy)(void *ref, void *dest, const void *src, size_t n);
    /*! Set all n bytes of ptr to ch */
    void * (*mu_memset)(void *ref, void *ptr, int ch, size_t n);
} sah_Mem_Util;


/* REQ-S2LRD-PINTFC-COA-UCO-001 */
/*!
 * User Context Object
 */
typedef struct fsl_shw_uco_t
{
    int           sahara_openfd;    /*!< this should be kernel-only?? */
    sah_Mem_Util* mem_util;     /*!< Memory utility fns  */
    uint32_t      user_ref;     /*!< User's reference */
    void (*callback)(struct fsl_shw_uco_t* uco); /*!< User's callback fn  */
    uint32_t      flags;             /*!< from fsl_shw_user_ctx_flags_t */
    unsigned      pool_size;    /*!< maximum size of user pool */
#ifdef __KERNEL__
    sah_Queue     result_pool;  /*!< where non-blocking results go */
    os_process_handle_t  process; /*!< remember for signalling User mode */
#else
    struct fsl_shw_uco_t* next; /*!< To allow user-mode chaining of contexts,
                                 for signalling.  */
#endif
} fsl_shw_uco_t;


/* REQ-S2LRD-PINTFC-API-GEN-006  ??  */
/*!
 * Result object
 */
typedef struct fsl_shw_result_t
{
    uint32_t         user_ref;
    fsl_shw_return_t code;
    uint32_t         detail1;
    uint32_t         detail2;
    sah_Head_Desc*   user_desc;
} fsl_shw_result_t;


/* REQ-S2LRD-PINTFC-COA-SKO-001 */
/*!
 * Secret Key Context Object
 */
typedef struct fsl_shw_sko_t
{
    uint32_t          flags;
    fsl_shw_key_alg_t algorithm;
    key_userid_t      userid;
    uint32_t          handle;
    uint16_t          key_length;
    uint8_t           key[64];
} fsl_shw_sko_t;


/* REQ-S2LRD-PINTFC-COA-CO-001 */
/*!
 * @brief Platform Capability Object
 */
typedef struct fsl_shw_pco_t
{  /* Consider turning these constants into symbols */
    int api_major;
    int api_minor;
    int driver_major;
    int driver_minor;
    fsl_shw_key_alg_t  sym_algorithms[4];
    fsl_shw_sym_mode_t sym_modes[4];
    fsl_shw_hash_alg_t hash_algorithms[4];
    uint8_t sym_support[5][4];  /* indexed by key alg then mode */
} fsl_shw_pco_t;


/* REQ-S2LRD-PINTFC-COA-HCO-001 */
/*!
 * Hash Context Object
 */
typedef struct fsl_shw_hco_t /* fsl_shw_hash_context_object */
{
    fsl_shw_hash_alg_t       algorithm;
    uint32_t                 flags;
    uint8_t                  digest_length; /* in bytes */
    uint8_t                  context_length; /* in bytes */
    uint8_t                  context_register_length; /* in bytes */
    uint32_t                 context[9]; /* largest digest + msg size */
} fsl_shw_hco_t;


/*!
 * HMAC Context Object
 */
typedef struct fsl_shw_hmco_t /* fsl_shw_hmac_context_object */
{
    fsl_shw_hash_alg_t       algorithm;
    uint32_t                 flags;
    uint8_t                  digest_length; /*!< in bytes */
    uint8_t                  context_length; /*!< in bytes */
    uint8_t                  context_register_length; /*!< in bytes */
    uint32_t                 ongoing_context[9]; /*!< largest digest + msg
                                                    size */
    uint32_t                 inner_precompute[9]; /*!< largest digest + msg
                                                      size */
    uint32_t                 outer_precompute[9]; /*!< largest digest + msg
                                                      size */
} fsl_shw_hmco_t;


/* REQ-S2LRD-PINTFC-COA-SCCO-001 */
/*!
 * Symmetric Crypto Context Object Context Object
 */
typedef struct fsl_shw_scco_t
{
    uint32_t                flags;
    unsigned                block_size_bytes; /* double duty block&ctx size */
    fsl_shw_sym_mode_t      mode;
    /* Could put modulus plus 16-octet context in union with arc4
       sbox+ptrs... */
    fsl_shw_ctr_mod_t       modulus_exp;
    uint8_t                 context[259];
} fsl_shw_scco_t;


/*!
 * Authenticate-Cipher Context Object

 * An object for controlling the function of, and holding information about,
 * data for the authenticate-cipher functions, #fsl_shw_gen_encrypt() and
 * #fsl_shw_auth_decrypt().
 */
typedef struct fsl_shw_acco_t
{
    uint32_t                 flags; /*!< See #fsl_shw_auth_ctx_flags_t for
                                       meanings */
    fsl_shw_acc_mode_t       mode; /*!< CCM only */
    uint8_t                  mac_length; /*!< User's value for length  */
    unsigned                 q_length; /*!< NIST parameter - */
    fsl_shw_scco_t           cipher_ctx_info; /*!< For running
                                                 encrypt/decrypt. */
    union {
        fsl_shw_scco_t       CCM_ctx_info; /*!< For running the CBC in
                                              AES-CCM.  */
        fsl_shw_hco_t        hash_ctx_info; /*!< For running the hash */
    } auth_info;                /*!< "auth" info struct  */
    uint8_t                  unencrypted_mac[16]; /*!< max block size... */
} fsl_shw_acco_t;


/*!
 *  Used by Sahara API to retrieve completed non-blocking results.
 */
typedef struct sah_results {
    unsigned         requested; /*!< number of results requested */
    unsigned         *actual;   /*!< number of results obtained */
    fsl_shw_result_t *results;  /*!< pointer to memory to hold results */
}  sah_results;


/******************************************************************************
 * Access Macros for Objects
 *****************************************************************************/
/*!
 * Get FSL SHW API version
 *
 * @param      pcobject  The Platform Capababilities Object to query.
 * @param[out] pcmajor   A pointer to where the major version
 *                       of the API is to be stored.
 * @param[out] pcminor   A pointer to where the minor version
 *                       of the API is to be stored.
 */
#define fsl_shw_pco_get_version(pcobject, pcmajor, pcminor)                   \
{                                                                             \
    *(pcmajor) = (pcobject)->api_major;                                       \
    *(pcminor) = (pcobject)->api_minor;                                       \
}


/*!
 * Get underlying driver version.
 *
 * @param      pcobject  The Platform Capababilities Object to query.
 * @param[out] pcmajor   A pointer to where the major version
 *                       of the driver is to be stored.
 * @param[out] pcminor   A pointer to where the minor version
 *                       of the driver is to be stored.
 */
#define fsl_shw_pco_get_driver_version(pcobject, pcmajor, pcminor)            \
{                                                                             \
    *(pcmajor) = (pcobject)->driver_major;                                    \
    *(pcminor) = (pcobject)->driver_minor;                                    \
}


/*!
 * Get list of symmetric algorithms supported.
 *
 * @param pcobject           The Platform Capababilities Object to query.
 * @param[out] pcalgorithms  A pointer to where to store the location of
 *                           the list of algorithms.
 * @param[out] pcacount      A pointer to where to store the number of
 *                           algorithms in the list at @a algorithms.
 */
#define fsl_shw_pco_get_sym_algorithms(pcobject, pcalgorithms, pcacount)      \
{                                                                             \
    *(pcalgorithms) = (pcobject)->sym_algorithms;                             \
    *(pcacount) = sizeof((pcobject)->sym_algorithms)/4;                       \
}


/*!
 * Get list of symmetric modes supported.
 *
 * @param pcobject        The Platform Capababilities Object to query.
 * @param[out] gsmodes    A pointer to where to store the location of
 *                        the list of modes.
 * @param[out] gsacount   A pointer to where to store the number of
 *                        algorithms in the list at @a modes.
 */
#define fsl_shw_pco_get_sym_modes(pcobject, gsmodes, gsacount)                \
{                                                                             \
    *(gsmodes) = (pcobject)->sym_modes;                                       \
    *(gsacount) = sizeof((pcobject)->sym_modes)/4;                            \
}


/*!
 * Get list of hash algorithms supported.
 *
 * @param pcobject           The Platform Capababilities Object to query.
 * @param[out] gsalgorithms  A pointer which will be set to the list of
 *                           algorithms.
 * @param[out] gsacount      The number of algorithms in the list at @a
 *                           algorithms.
 */
#define fsl_shw_pco_get_hash_algorithms(pcobject, gsalgorithms, gsacount)     \
{                                                                             \
    *(gsalgorithms) = (pcobject)->hash_algorithms;                            \
    *(gsacount) = sizeof((pcobject)->hash_algorithms)/4;                      \
}


/*!
 * Determine whether the combination of a given symmetric algorithm and a given
 * mode is supported.
 *
 * @param pcobject   The Platform Capababilities Object to query.
 * @param pcalg      A Symmetric Cipher algorithm.
 * @param pcmode     A Symmetric Cipher mode.
 *
 * @return 0 if combination is not supported, non-zero if supported.
 */
#define fsl_shw_pco_check_sym_supported(pcobject, pcalg, pcmode)              \
    ((pcobject)->sym_support[pcalg][pcmode])


/*!
 * Initialize a User Context Object.
 *
 * This function must be called before performing any other operation with the
 * Object.  It sets the User Context Object to initial values, and set the size
 * of the results pool.  The mode will be set to a default of
 * #FSL_UCO_BLOCKING_MODE.
 *
 * When using non-blocking operations, this sets the maximum number of
 * operations which can be outstanding.  This number includes the counts of
 * operations waiting to start, operation(s) being performed, and results which
 * have not been retrieved.
 *
 * Changes to this value are ignored once user registration has completed.  It
 * should be set to 1 if only blocking operations will ever be performed.
 *
 * @param ucontext     The User Context object to operate on.
 * @param usize        The maximum number of operations which can be
 *                     outstanding.
 */
#define fsl_shw_uco_init(ucontext, usize)                                     \
{                                                                             \
      (ucontext)->pool_size = usize;                                          \
      (ucontext)->flags = FSL_UCO_BLOCKING_MODE;                              \
      (ucontext)->sahara_openfd = -1;                                         \
      (ucontext)->mem_util = NULL;                                            \
      (ucontext)->callback = NULL;                                            \
}

/*!
 * Set the User Reference for the User Context.
 *
 * @param ucontext     The User Context object to operate on.
 * @param uref         A value which will be passed back with a result.
 */
#define fsl_shw_uco_set_reference(ucontext, uref)                             \
      (ucontext)->user_ref = uref


/*!
 * Set the User Reference for the User Context.
 *
 * @param ucontext     The User Context object to operate on.
 * @param ucallback    The function the API will invoke when an operation
 *                     completes.
 */
#define fsl_shw_uco_set_callback(ucontext, ucallback)                         \
      (ucontext)->callback = ucallback


/*!
 * Set flags in the User Context.
 *
 * Turns on the flags specified in @a flags.  Other flags are untouched.
 *
 * @param ucontext     The User Context object to operate on.
 * @param uflags       ORed values from #fsl_shw_user_ctx_flags_t.
 */
#define fsl_shw_uco_set_flags(ucontext, uflags)                               \
      (ucontext)->flags |= (uflags)


/*!
 * Clear flags in the User Context.
 *
 * Turns off the flags specified in @a flags.  Other flags are untouched.
 *
 * @param ucontext     The User Context object to operate on.
 * @param uflags       ORed values from #fsl_shw_user_ctx_flags_t.
 */
#define fsl_shw_uco_clear_flags(ucontext, uflags)                             \
      (ucontext)->flags &= ~(uflags)


/*!
 * Retrieve the reference value from a Result Object.
 *
 * @param robject  The result object to query.
 *
 * @return The reference associated with the request.
 */
#define fsl_shw_ro_get_reference(robject)                                    \
       (robject)->user_ref


/*!
 * Retrieve the status code from a Result Object.
 *
 * @param robject  The result object to query.
 *
 * @return The status of the request.
 */
#define fsl_shw_ro_get_status(robject)                                       \
       (robject)->code


/*!
 * Initialize a Secret Key Object.
 *
 * This function must be called before performing any other operation with
 * the Object.
 *
 * @param skobject     The Secret Key Object to be initialized.
 * @param skalgorithm  DES, AES, etc.
 *
 */
#define fsl_shw_sko_init(skobject,skalgorithm)                               \
{                                                                            \
       (skobject)->algorithm = skalgorithm;                                  \
       (skobject)->flags = 0;                                                \
}


/*!
 * Store a cleartext key in the key object.
 *
 * This has the side effect of setting the #FSL_SKO_KEY_PRESENT flag and
 * resetting the #FSL_SKO_KEY_ESTABLISHED flag.
 *
 * @param skobject     A variable of type #fsl_shw_sko_t.
 * @param skkey        A pointer to the beginning of the key.
 * @param skkeylen     The length, in octets, of the key.  The value should be
 *                     appropriate to the key size supported by the algorithm.
 *                     64 octets is the absolute maximum value allowed for this
 *                     call.
 */
#define fsl_shw_sko_set_key(skobject, skkey, skkeylen)                       \
{                                                                            \
       (skobject)->key_length = skkeylen;                                    \
       copy_bytes((skobject)->key, skkey, skkeylen);                         \
       (skobject)->flags |= FSL_SKO_KEY_PRESENT;                             \
       (skobject)->flags &= ~FSL_SKO_KEY_ESTABLISHED;                        \
}

/*!
 * Set a size for the key.
 *
 * This function would normally be used when the user wants the key to be
 * generated from a random source.
 *
 * @param skobject   A variable of type #fsl_shw_sko_t.
 * @param skkeylen   The length, in octets, of the key.  The value should be
 *                   appropriate to the key size supported by the algorithm.
 *                   64 octets is the absolute maximum value allowed for this
 *                   call.
 */
#define fsl_shw_sko_set_key_length(skobject, skkeylen)                       \
       (skobject)->key_length = skkeylen;


/*!
 * Set the User ID associated with the key.
 *
 * @param skobject   A variable of type #fsl_shw_sko_t.
 * @param skuserid   The User ID to identify authorized users of the key.
 */
#define fsl_shw_sko_set_user_id(skobject, skuserid)                           \
       (skobject)->userid = (skuserid)


/*!
 * Set the establish key handle into a key object.
 *
 * The @a userid field will be used to validate the access to the unwrapped
 * key.  This feature is not available for all platforms, nor for all
 * algorithms and modes.
 *
 * The #FSL_SKO_KEY_ESTABLISHED will be set (and the #FSL_SKO_KEY_PRESENT flag
 * will be cleared).
 *
 * @param skobject   A variable of type #fsl_shw_sko_t.
 * @param skuserid   The User ID to verify this user is an authorized user of
 *                   the key.
 * @param skhandle   A @a handle from #fsl_shw_sko_get_established_info.
 */
#define fsl_shw_sko_set_established_info(skobject, skuserid, skhandle)        \
{                                                                             \
       (skobject)->userid = (skuserid);                                       \
       (skobject)->handle = (skhandle);                                       \
       (skobject)->flags |= FSL_SKO_KEY_ESTABLISHED;                          \
       (skobject)->flags &=                                                   \
                       ~(FSL_SKO_KEY_PRESENT);   \
}


/*!
 * Retrieve the established-key handle from a key object.
 *
 * @param skobject   A variable of type #fsl_shw_sko_t.
 * @param skhandle   The location to store the @a handle of the unwrapped
 *                   key.
 */
#define fsl_shw_sko_get_established_info(skobject, skhandle)                  \
       *(skhandle) = (skobject)->handle


/*!
 * Extract the algorithm from a key object.
 *
 * @param      skobject     The Key Object to be queried.
 * @param[out] skalgorithm  A pointer to the location to store the algorithm.
 */
#define fsl_shw_sko_get_algorithm(skobject, skalgorithm)                      \
       *(skalgorithm) = (skobject)->algorithm


/*!
 * Determine the size of a wrapped key based upon the cleartext key's length.
 *
 * This function can be used to calculate the number of octets that
 * #fsl_shw_extract_key() will write into the location at @a covered_key.
 *
 * If zero is returned at @a length, this means that the key length in
 * @a key_info is not supported.
 *
 * @param      wkeyinfo         Information about a key to be wrapped.
 * @param      wkeylen          Location to store the length of a wrapped
 *                              version of the key in @a key_info.
 */
#define fsl_shw_sko_calculate_wrapped_size(wkeyinfo, wkeylen)                 \
{                                                                             \
     if ((wkeyinfo)->key_length > 32) {                                       \
         *(wkeylen) = 0;                                                      \
     } else {                                                                 \
         *(wkeylen) = 66;                                                     \
     }                                                                        \
}


/*!
 * Set some flags in the key object.
 *
 * Turns on the flags specified in @a flags.  Other flags are untouched.
 *
 * @param skobject     A variable of type #fsl_shw_sko_t.
 * @param skflags      (One or more) ORed members of #fsl_shw_key_flags_t which
 *                     are to be set.
 */
#define fsl_shw_sko_set_flags(skobject, skflags)                              \
      (skobject)->flags |= (skflags)


/*!
 * Clear some flags in the key object.
 *
 * Turns off the flags specified in @a flags.  Other flags are untouched.
 *
 * @param skobject      A variable of type #fsl_shw_sko_t.
 * @param skflags       (One or more) ORed members of #fsl_shw_key_flags_t
 *                      which are to be reset.
 */
#define fsl_shw_sko_clear_flags(skobject, skflags)                            \
      (skobject)->flags &= ~(skflags)


/*!
 * Initialize a Hash Context Object.
 *
 * This function must be called before performing any other operation with the
 * Object.  It sets the current message length and hash algorithm in the hash
 * context object.
 *
 * @param      hcobject    The hash context to operate upon.
 * @param      hcalgorithm The hash algorithm to be used (#FSL_HASH_ALG_MD5,
 *                         #FSL_HASH_ALG_SHA256, etc).
 *
 */
#define fsl_shw_hco_init(hcobject, hcalgorithm)                               \
{                                                                             \
     (hcobject)->algorithm = hcalgorithm;                                     \
     (hcobject)->flags = 0;                                                   \
     switch (hcalgorithm) {                                                   \
     case FSL_HASH_ALG_MD5:                                                   \
         (hcobject)->digest_length = 16;                                      \
         (hcobject)->context_length = 16;                                     \
         (hcobject)->context_register_length = 24;                            \
         break;                                                               \
     case FSL_HASH_ALG_SHA1:                                                  \
         (hcobject)->digest_length = 20;                                      \
         (hcobject)->context_length = 20;                                     \
         (hcobject)->context_register_length = 24;                            \
         break;                                                               \
     case FSL_HASH_ALG_SHA224:                                                \
         (hcobject)->digest_length = 28;                                      \
         (hcobject)->context_length = 32;                                     \
         (hcobject)->context_register_length = 36;                            \
         break;                                                               \
     case FSL_HASH_ALG_SHA256:                                                \
         (hcobject)->digest_length = 32;                                      \
         (hcobject)->context_length = 32;                                     \
         (hcobject)->context_register_length = 36;                            \
         break;                                                               \
     default:                                                                 \
         /* error ! */                                                        \
         (hcobject)->digest_length = 1;                                       \
         (hcobject)->context_length = 1;                                      \
         (hcobject)->context_register_length = 1;                             \
         break;                                                               \
     }                                                                        \
}


/*!
 * Get the current hash value and message length from the hash context object.
 *
 * The algorithm must have already been specified.  See #fsl_shw_hco_init().
 *
 * @param      hcobject   The hash context to query.
 * @param[out] hccontext  Pointer to the location of @a length octets where to
 *                        store a copy of the current value of the digest.
 * @param      hcclength  Number of octets of hash value to copy.
 * @param[out] hcmsglen   Pointer to the location to store the number of octets
 *                        already hashed.
 */
#define fsl_shw_hco_get_digest(hcobject, hccontext, hcclength, hcmsglen)      \
{                                                                             \
     copy_bytes(hccontext, (hcobject)->context, hcclength);                   \
         if ((hcobject)->algorithm == FSL_HASH_ALG_SHA224                     \
             || (hcobject)->algorithm == FSL_HASH_ALG_SHA256) {               \
             *(hcmsglen) = (hcobject)->context[8];                            \
         } else {                                                             \
             *(hcmsglen) = (hcobject)->context[5];                            \
         }                                                                    \
}


/*!
 * Get the hash algorithm from the hash context object.
 *
 * @param      hcobject    The hash context to query.
 * @param[out] hcalgorithm Pointer to where the algorithm is to be stored.
 */
#define fsl_shw_hco_get_info(hcobject, hcalgorithm)                           \
{                                                                             \
     *(hcalgorithm) = (hcobject)->algorithm;                                  \
}


/*!
 * Set the current hash value and message length in the hash context object.
 *
 * The algorithm must have already been specified.  See #fsl_shw_hco_init().
 *
 * @param      hcobject  The hash context to operate upon.
 * @param      hccontext Pointer to buffer of appropriate length to copy into
 *                       the hash context object.
 * @param      hcmsglen  The number of octets of the message which have
 *                        already been hashed.
 *
 */
#define fsl_shw_hco_set_digest(hcobject, hccontext, hcmsglen)                 \
{                                                                             \
     copy_bytes((hcobject)->context, hccontext, (hcobject)->context_length);  \
     if (((hcobject)->algorithm == FSL_HASH_ALG_SHA224)                       \
         || ((hcobject)->algorithm == FSL_HASH_ALG_SHA256)) {                 \
         (hcobject)->context[8] = hcmsglen;                                   \
     } else {                                                                 \
         (hcobject)->context[5] = hcmsglen;                                   \
     }                                                                        \
}


/*!
 * Set flags in a Hash Context Object.
 *
 * Turns on the flags specified in @a flags.  Other flags are untouched.
 *
 * @param hcobject   The hash context to be operated on.
 * @param hcflags    The flags to be set in the context.  These can be ORed
 *                   members of #fsl_shw_hash_ctx_flags_t.
 */
#define fsl_shw_hco_set_flags(hcobject, hcflags)                              \
      (hcobject)->flags |= (hcflags)


/*!
 * Clear flags in a Hash Context Object.
 *
 * Turns off the flags specified in @a flags.  Other flags are untouched.
 *
 * @param hcobject   The hash context to be operated on.
 * @param hcflags    The flags to be reset in the context.  These can be ORed
 *                   members of #fsl_shw_hash_ctx_flags_t.
 */
#define fsl_shw_hco_clear_flags(hcobject, hcflags)                            \
      (hcobject)->flags &= ~(hcflags)



/*!
 * Initialize an HMAC Context Object.
 *
 * This function must be called before performing any other operation with the
 * Object.  It sets the current message length and hash algorithm in the HMAC
 * context object.
 *
 * @param      hcobject    The HMAC context to operate upon.
 * @param      hcalgorithm The hash algorithm to be used (#FSL_HASH_ALG_MD5,
 *                         #FSL_HASH_ALG_SHA256, etc).
 *
 */
#define fsl_shw_hmco_init(hcobject, hcalgorithm)                              \
    fsl_shw_hco_init(hcobject, hcalgorithm)


/*!
 * Set flags in an HMAC Context Object.
 *
 * Turns on the flags specified in @a flags.  Other flags are untouched.
 *
 * @param hcobject   The HMAC context to be operated on.
 * @param hcflags    The flags to be set in the context.  These can be ORed
 *                   members of #fsl_shw_hmac_ctx_flags_t.
 */
#define fsl_shw_hmco_set_flags(hcobject, hcflags)                             \
      (hcobject)->flags |= (hcflags)


/*!
 * Clear flags in an HMAC Context Object.
 *
 * Turns off the flags specified in @a flags.  Other flags are untouched.
 *
 * @param hcobject   The HMAC context to be operated on.
 * @param hcflags    The flags to be reset in the context.  These can be ORed
 *                   members of #fsl_shw_hmac_ctx_flags_t.
 */
#define fsl_shw_hmco_clear_flags(hcobject, hcflags)                           \
      (hcobject)->flags &= ~(hcflags)


/*!
 * Initialize a Symmetric Cipher Context Object.
 *
 * This function must be called before performing any other operation with the
 * Object.  This will set the @a mode and @a algorithm and initialize the
 * Object.
 *
 * @param scobject  The context object to operate on.
 * @param scalg     The cipher algorithm this context will be used with.
 * @param scmode    #FSL_SYM_MODE_CBC, #FSL_SYM_MODE_ECB, etc.
 *
 */
#define fsl_shw_scco_init(scobject, scalg, scmode)                            \
{                                                                             \
      register uint32_t bsb;   /* block-size bytes */                         \
                                                                              \
      switch (scalg) {                                                        \
      case FSL_KEY_ALG_AES:                                                   \
          bsb = 16;                                                           \
          break;                                                              \
      case FSL_KEY_ALG_DES:                                                   \
          /* fall through */                                                  \
      case FSL_KEY_ALG_TDES:                                                  \
          bsb = 8;                                                            \
          break;                                                              \
      case FSL_KEY_ALG_ARC4:                                                  \
          bsb = 259;                                                          \
          break;                                                              \
      case FSL_KEY_ALG_HMAC:                                                  \
          bsb = 1;  /* meaningless */                                         \
          break;                                                              \
      default:                                                                \
          bsb = 00;                                                           \
      }                                                                       \
      (scobject)->block_size_bytes = bsb;                                     \
      (scobject)->mode = scmode;                                              \
      (scobject)->flags = 0;                                                  \
}


/*!
 * Set the flags for a Symmetric Cipher Context.
 *
 * Turns on the flags specified in @a flags.  Other flags are untouched.
 *
 * @param scobject The context object to operate on.
 * @param scflags  The flags to reset (one or more values from
 *                 #fsl_shw_sym_ctx_flags_t ORed together).
 *
 */
#define fsl_shw_scco_set_flags(scobject, scflags)                             \
       (scobject)->flags |= (scflags)


/*!
 * Clear some flags in a Symmetric Cipher Context Object.
 *
 * Turns off the flags specified in @a flags.  Other flags are untouched.
 *
 * @param scobject The context object to operate on.
 * @param scflags  The flags to reset (one or more values from
 *                 #fsl_shw_sym_ctx_flags_t ORed together).
 *
 */
#define fsl_shw_scco_clear_flags(scobject, scflags)                           \
       (scobject)->flags &= ~(scflags)


/*!
 * Set the Context (IV) for a Symmetric Cipher Context.
 *
 * This is to set the context/IV for #FSL_SYM_MODE_CBC mode, or to set the
 * context (the S-Box and pointers) for ARC4.  The full context size will
 * be copied.
 *
 * @param scobject  The context object to operate on.
 * @param sccontext A pointer to the buffer which contains the context.
 *
 */
#define fsl_shw_scco_set_context(scobject, sccontext)                         \
       copy_bytes((scobject)->context, sccontext,                             \
                  (scobject)->block_size_bytes);

/*!
 * Get the Context for a Symmetric Cipher Context.
 *
 * This is to retrieve the context/IV for #FSL_SYM_MODE_CBC mode, or to
 * retrieve context (the S-Box and pointers) for ARC4.  The full context
 * will be copied.
 *
 * @param      scobject  The context object to operate on.
 * @param[out] sccontext Pointer to location where context will be stored.
 */
#define fsl_shw_scco_get_context(scobject, sccontext)                         \
       copy_bytes(sccontext, (scobject)->context, (scobject)->block_size_bytes)


/*!
 * Set the Counter Value for a Symmetric Cipher Context.
 *
 * This will set the Counter Value for CTR mode.
 *
 * @param scobject  The context object to operate on.
 * @param sccounter The starting counter value.  The number of octets.
 *                  copied will be the block size for the algorithm.
 * @param scmodulus The modulus for controlling the incrementing of the
 *                  counter.
 *
 */
#define fsl_shw_scco_set_counter_info(scobject, sccounter, scmodulus)        \
       {                                                                     \
           if ((sccounter) != NULL) {                                        \
               copy_bytes((scobject)->context, sccounter,                    \
                          (scobject)->block_size_bytes);                     \
           }                                                                 \
           (scobject)->modulus_exp = scmodulus;                              \
       }


/*!
 * Get the Counter Value for a Symmetric Cipher Context.
 *
 * This will retrieve the Counter Value is for CTR mode.
 *
 * @param     scobject    The context object to query.
 * @param[out] sccounter  Pointer to location to store the current counter
 *                        value.  The number of octets copied will be the
 *                        block size for the algorithm.
 * @param[out] scmodulus  Pointer to location to store the modulus.
 *
 */
#define fsl_shw_scco_get_counter_info(scobject, sccounter, scmodulus)        \
       {                                                                     \
           if ((sccounter) != NULL) {                                        \
               copy_bytes(sccounter, (scobject)->context,                    \
                          (scobject)->block_size_bytes);                     \
           }                                                                 \
           if ((scmodulus) != NULL) {                                        \
               *(scmodulus) = (scobject)->modulus_exp;                       \
           }                                                                 \
       }


/*!
 * Initialize a Authentication-Cipher Context.
 *
 * @param acobject  Pointer to object to operate on.
 * @param acmode    The mode for this object (only #FSL_ACC_MODE_CCM
 *                  supported).
 */
#define fsl_shw_acco_init(acobject, acmode)                                   \
   {                                                                          \
       (acobject)->flags = 0;                                                 \
       (acobject)->mode = (acmode);                                           \
   }


/*!
 * Set the flags for a Authentication-Cipher Context.
 *
 * Turns on the flags specified in @a flags.  Other flags are untouched.
 *
 * @param acobject  Pointer to object to operate on.
 * @param acflags   The flags to set (one or more from
 *                  #fsl_shw_auth_ctx_flags_t ORed together).
 *
 */
#define fsl_shw_acco_set_flags(acobject, acflags)                             \
       (acobject)->flags |= (acflags)


/*!
 * Clear some flags in a Authentication-Cipher Context Object.
 *
 * Turns off the flags specified in @a flags.  Other flags are untouched.
 *
 * @param acobject  Pointer to object to operate on.
 * @param acflags   The flags to reset (one or more from
 *                  #fsl_shw_auth_ctx_flags_t ORed together).
 *
 */
#define fsl_shw_acco_clear_flags(acobject, acflags)                           \
       (acobject)->flags &= ~(acflags)


/*!
 * Set up the Authentication-Cipher Object for CCM mode.
 *
 * This will set the @a auth_object for CCM mode and save the @a ctr,
 * and @a mac_length.  This function can be called instead of
 * #fsl_shw_acco_init().
 *
 * The paramater @a ctr is Counter Block 0, (counter value 0), which is for the
 * MAC.
 *
 * @param acobject  Pointer to object to operate on.
 * @param acalg     Cipher algorithm.  Only AES is supported.
 * @param accounter The initial counter value.
 * @param acmaclen  The number of octets used for the MAC.  Valid values are
 *                  4, 6, 8, 10, 12, 14, and 16.
 */
/* Do we need to stash the +1 value of the CTR somewhere? */
#define fsl_shw_acco_set_ccm(acobject, acalg, accounter, acmaclen)            \
{                                                                             \
      (acobject)->flags = 0;                                                  \
      (acobject)->mode = FSL_ACC_MODE_CCM;                                    \
      (acobject)->auth_info.CCM_ctx_info.block_size_bytes = 16;               \
      (acobject)->cipher_ctx_info.block_size_bytes = 16;                      \
      (acobject)->mac_length = acmaclen;                                      \
      fsl_shw_scco_set_counter_info(&(acobject)->cipher_ctx_info, accounter,  \
            FSL_CTR_MOD_128);                                                 \
}


/*!
 * Format the First Block (IV) & Initial Counter Value per NIST CCM.
 *
 * This function will also set the IV and CTR values per Appendix A of NIST
 * Special Publication 800-38C (May 2004).  It will also perform the
 * #fsl_shw_acco_set_ccm() operation with information derived from this set of
 * parameters.
 *
 * Note this function assumes the algorithm is AES.  It initializes the
 * @a auth_object by setting the mode to #FSL_ACC_MODE_CCM and setting the
 * flags to be #FSL_ACCO_NIST_CCM.
 *
 * @param acobject  Pointer to object to operate on.
 * @param act       The number of octets used for the MAC.  Valid values are
 *                  4, 6, 8, 10, 12, 14, and 16.
 * @param acad      Number of octets of Associated Data (may be zero).
 * @param acq       A value for the size of the length of @a q field.  Valid
 *                  values are 1-8.
 * @param acN       The Nonce (packet number or other changing value). Must
 *                  be (15 - @a q_length) octets long.
 * @param acQ       The value of Q (size of the payload in octets).
 *
 */
/* Do we need to stash the +1 value of the CTR somewhere? */
#define fsl_shw_ccm_nist_format_ctr_and_iv(acobject, act, acad, acq, acN, acQ)\
    {                                                                         \
        uint64_t Q = acQ;                                                     \
        uint8_t bflag = ((acad)?0x40:0) | ((((act)-2)/2)<<3) | ((acq)-1);     \
        unsigned i;                                                           \
        uint8_t* qptr = (acobject)->auth_info.CCM_ctx_info.context + 15;      \
        (acobject)->auth_info.CCM_ctx_info.block_size_bytes = 16;             \
        (acobject)->cipher_ctx_info.block_size_bytes = 16;                    \
        (acobject)->mode = FSL_ACC_MODE_CCM;                                  \
        (acobject)->flags  = FSL_ACCO_NIST_CCM;                               \
                                                                              \
        /* Store away the MAC length (after calculating actual value */       \
        (acobject)->mac_length = (act);                                       \
        /* Set Flag field in Block 0 */                                       \
        *((acobject)->auth_info.CCM_ctx_info.context) = bflag;                \
        /* Set Nonce field in Block 0 */                                      \
        copy_bytes((acobject)->auth_info.CCM_ctx_info.context+1, acN,         \
                   15-(acq));                                                 \
        /* Set Flag field in ctr */                                           \
        *((acobject)->cipher_ctx_info.context) = (acq)-1;                     \
        /* Update the Q (payload length) field of Block0 */                   \
        (acobject)->q_length = acq;                                           \
        for (i = 0; i < (acq); i++) {                                         \
            *qptr-- = Q & 0xFF;                                               \
            Q >>= 8;                                                          \
        }                                                                     \
        /* Set the Nonce field of the ctr */                                  \
        copy_bytes((acobject)->cipher_ctx_info.context+1, acN, 15-(acq));     \
        /* Clear the block counter field of the ctr */                        \
        memset((acobject)->cipher_ctx_info.context+16-(acq), 0, (acq)+1);     \
     }


/*!
 * Update the First Block (IV) & Initial Counter Value per NIST CCM.
 *
 * This function will set the IV and CTR values per Appendix A of NIST Special
 * Publication 800-38C (May 2004).
 *
 * Note this function assumes that #fsl_shw_ccm_nist_format_ctr_and_iv() has
 * previously been called on the @a auth_object.
 *
 * @param acobject  Pointer to object to operate on.
 * @param acN       The Nonce (packet number or other changing value). Must
 *                  be (15 - @a q_length) octets long.
 * @param acQ       The value of Q (size of the payload in octets).
 *
 */
/* Do we need to stash the +1 value of the CTR somewhere? */
#define fsl_shw_ccm_nist_update_ctr_and_iv(acobject, acN, acQ)                \
     {                                                                        \
        uint64_t Q = acQ;                                                     \
        unsigned i;                                                           \
        uint8_t* qptr = (acobject)->auth_info.CCM_ctx_info.context + 15;      \
                                                                              \
        /* Update the Nonce field field of Block0 */                          \
        copy_bytes((acobject)->auth_info.CCM_ctx_info.context+1, acN,         \
               15 - (acobject)->q_length);                                    \
        /* Update the Q (payload length) field of Block0 */                   \
        for (i = 0; i < (acobject)->q_length; i++) {                          \
            *qptr-- = Q & 0xFF;                                               \
            Q >>= 8;                                                          \
        }                                                                     \
        /* Update the Nonce field of the ctr */                               \
        copy_bytes((acobject)->cipher_ctx_info.context+1, acN,                \
               15 - (acobject)->q_length);                                    \
     }


/******************************************************************************
 * Library functions
 *****************************************************************************/
/* REQ-S2LRD-PINTFC-API-GEN-003 */
extern fsl_shw_pco_t* fsl_shw_get_capabilities(fsl_shw_uco_t* user_ctx);


/* REQ-S2LRD-PINTFC-API-GEN-004 */
extern fsl_shw_return_t fsl_shw_register_user(fsl_shw_uco_t* user_ctx);


/* REQ-S2LRD-PINTFC-API-GEN-005 */
extern fsl_shw_return_t fsl_shw_deregister_user(fsl_shw_uco_t* user_ctx);


/* REQ-S2LRD-PINTFC-API-GEN-006 */
extern fsl_shw_return_t fsl_shw_get_results(fsl_shw_uco_t* user_ctx,
                                            unsigned result_size,
                                            fsl_shw_result_t results[],
                                            unsigned* result_count);

extern fsl_shw_return_t fsl_shw_establish_key(
                                fsl_shw_uco_t* user_ctx,
                                fsl_shw_sko_t* key_info,
                                fsl_shw_key_wrap_t establish_type,
                                const uint8_t* key);

extern fsl_shw_return_t fsl_shw_extract_key(fsl_shw_uco_t* user_ctx,
                                fsl_shw_sko_t* key_info,
                                uint8_t* covered_key);

extern fsl_shw_return_t fsl_shw_release_key(
                                fsl_shw_uco_t* user_ctx,
                                fsl_shw_sko_t* key_info);


/* REQ-S2LRD-PINTFC-API-BASIC-SYM-002 */
/* PINTFC-API-BASIC-SYM-ARC4-001 */
/* PINTFC-API-BASIC-SYM-ARC4-002 */
extern fsl_shw_return_t fsl_shw_symmetric_encrypt(
                                fsl_shw_uco_t* user_ctx,
                                fsl_shw_sko_t* key_info,
                                fsl_shw_scco_t* sym_ctx,
                                uint32_t length,
                                const uint8_t* pt,
                                uint8_t* ct);

/* PINTFC-API-BASIC-SYM-002 */
/* PINTFC-API-BASIC-SYM-ARC4-001 */
/* PINTFC-API-BASIC-SYM-ARC4-002 */
extern fsl_shw_return_t fsl_shw_symmetric_decrypt(
                                fsl_shw_uco_t* user_ctx,
                                fsl_shw_sko_t* key_info,
                                fsl_shw_scco_t* sym_ctx,
                                uint32_t length,
                                const uint8_t* ct,
                                uint8_t* pt);


/* REQ-S2LRD-PINTFC-API-BASIC-HASH-005 */
extern fsl_shw_return_t fsl_shw_hash(
                                fsl_shw_uco_t* user_ctx,
                                fsl_shw_hco_t* hash_ctx,
                                const uint8_t* msg,
                                uint32_t length,
                                uint8_t* result,
                                uint32_t result_len);


/* REQ-S2LRD-PINTFC-API-BASIC-HMAC-001 */
extern fsl_shw_return_t fsl_shw_hmac_precompute(
                                fsl_shw_uco_t* user_ctx,
                                fsl_shw_sko_t* key_info,
                                fsl_shw_hmco_t* hmac_ctx);


/* REQ-S2LRD-PINTFC-API-BASIC-HMAC-002 */
extern fsl_shw_return_t fsl_shw_hmac(
                                fsl_shw_uco_t* user_ctx,
                                fsl_shw_sko_t* key_info,
                                fsl_shw_hmco_t* hmac_ctx,
                                const uint8_t* msg,
                                uint32_t length,
                                uint8_t* result,
                                uint32_t result_len);


/* REQ-S2LRD-PINTFC-API-BASIC-RNG-002 */
extern fsl_shw_return_t fsl_shw_get_random(
                                fsl_shw_uco_t* user_ctx,
                                uint32_t length,
                                uint8_t* data);


extern fsl_shw_return_t fsl_shw_gen_encrypt(
                                fsl_shw_uco_t* user_ctx,
                                fsl_shw_acco_t* auth_ctx,
                                fsl_shw_sko_t* cipher_key_info,
                                fsl_shw_sko_t* auth_key_info,
                                uint32_t auth_data_length,
                                const uint8_t* auth_data,
                                uint32_t payload_length,
                                const uint8_t* payload,
                                uint8_t* ct,
                                uint8_t* auth_value);


extern fsl_shw_return_t fsl_shw_auth_decrypt(
                                fsl_shw_uco_t* user_ctx,
                                fsl_shw_acco_t* auth_ctx,
                                fsl_shw_sko_t* cipher_key_info,
                                fsl_shw_sko_t* auth_key_info,
                                uint32_t auth_data_length,
                                const uint8_t* auth_data,
                                uint32_t payload_length,
                                const uint8_t* ct,
                                const uint8_t* auth_value,
                                uint8_t* payload);

#if 0
sah_Test_Status sah_Check_Test_Mode(void);
#endif

fsl_shw_return_t sah_Append_Desc (
    const sah_Mem_Util *mu,
    sah_Head_Desc **desc_head,
    const uint32_t header,
    sah_Link *link1,
    sah_Link *link2);

/* Utility Function leftover from sahara1 API */
void sah_Descriptor_Chain_Destroy(
    const sah_Mem_Util    * mu,
    sah_Head_Desc        ** desc_chain);

/* Utility Function leftover from sahara1 API */
fsl_shw_return_t sah_Descriptor_Chain_Execute(
    sah_Head_Desc *desc_chain,
    fsl_shw_uco_t *user_ctx);

fsl_shw_return_t sah_Append_Link(
    const sah_Mem_Util *mu,
    sah_Link *link,
    uint8_t *p,
    const size_t length,
    const sah_Link_Flags flags);

fsl_shw_return_t sah_Create_Link(
    const sah_Mem_Util *mu,
    sah_Link **link,
    uint8_t *p,
    const size_t length,
    const sah_Link_Flags flags);

fsl_shw_return_t sah_Create_Key_Link(
    const sah_Mem_Util *mu,
    sah_Link **link,
    fsl_shw_sko_t* key_info);

void sah_Destroy_Link(
    const sah_Mem_Util *mu,
    sah_Link * link);

void sah_Postprocess_Results(
    fsl_shw_uco_t* user_ctx,
    sah_results* result_info);


#endif /* SAHARA2_API_H */
