#ifndef BCE_QUEUE_H
#define BCE_QUEUE_H

#include <linux/completion.h>
#include <linux/pci.h>

#define BCE_CMD_SIZE 0x40

struct bce_device;

enum bce_queue_type {
    BCE_QUEUE_CQ, BCE_QUEUE_SQ
};
struct bce_queue {
    int qid;
    int type;
};
struct bce_queue_cq {
    int qid;
    int type;
    u32 el_count;
    dma_addr_t dma_handle;
    void *data;

    u32 index;
};
struct bce_queue_sq;
typedef void (*bce_sq_completion)(struct bce_queue_sq *q);
struct bce_sq_completion_data {
    u32 status;
    u64 data_size;
    u64 result;
};
struct bce_queue_sq {
    int qid;
    int type;
    u32 el_size;
    u32 el_count;
    dma_addr_t dma_handle;
    void *data;
    void *userdata;

    u32 completion_cidx, completion_tail;
    struct bce_sq_completion_data *completion_data;
    bool has_pending_completions;
    bce_sq_completion completion;
};

struct bce_queue_cmdq_result_el {
    struct completion cmpl;
    u32 status;
    u64 result;
};
struct bce_queue_cmdq {
    struct bce_queue_sq *sq;
    void __iomem *reg_mem_dma;
    struct spinlock lck;
    u32 head, tail;
    int nospace_cntr;
    struct completion nospace_cmpl;
    struct bce_queue_cmdq_result_el **tres;
};

struct bce_queue_memcfg {
    u16 qid;
    u16 el_count;
    u16 vector_or_cq;
    u16 _pad;
    u64 addr;
    u64 length;
};

enum bce_qe_completion_status {
    BCE_COMPLETION_SUCCESS = 0,
    BCE_COMPLETION_ERROR = 1,
    BCE_COMPLETION_ABORTED = 2,
    BCE_COMPLETION_NO_SPACE = 3,
    BCE_COMPLETION_OVERRUN = 4
};
enum bce_qe_completion_flags {
    BCE_COMPLETION_FLAG_PENDING = 0x8000
};
struct bce_qe_completion {
    u64 data_size;
    u64 result;
    u16 qid;
    u16 completion_index;
    u16 status; // bce_qe_completion_status
    u16 flags;  // bce_qe_completion_flags
};

enum bce_cmdq_command {
    BCE_CMD_REGISTER_MEMORY_QUEUE = 0x20,
    BCE_CMD_UNREGISTER_MEMORY_QUEUE = 0x30,
    BCE_CMD_FLUSH_MEMORY_QUEUE = 0x40,
    BCE_CMD_SET_MEMORY_QUEUE_PROPERTY = 0x50
};
struct bce_cmdq_simple_memory_queue_cmd {
    u16 cmd; // bce_cmdq_command
    u16 flags;
    u16 qid;
};
struct bce_cmdq_register_memory_queue_cmd {
    u16 cmd; // bce_cmdq_command
    u16 flags;
    u16 qid;
    u16 _pad;
    u16 el_count;
    u16 vector_or_cq;
    u16 _pad2;
    u16 name_len;
    char name[0x20];
    u64 addr;
    u64 length;
};

static __always_inline void *bce_sq_element(struct bce_queue_sq *q, int i) {
    return (void *) ((u8 *) q->data + q->el_size * i);
}
static __always_inline void *bce_cq_element(struct bce_queue_cq *q, int i) {
    return (void *) ((struct bce_qe_completion *) q->data + i);
}

static __always_inline struct bce_sq_completion_data *bce_next_completion(struct bce_queue_sq *sq) {
    rmb();
    if (sq->completion_cidx == sq->completion_tail)
        return NULL;
    sq->completion_cidx = (sq->completion_cidx + 1) % sq->el_count;
    return &sq->completion_data[sq->completion_cidx];
}

struct bce_queue_cq *bce_alloc_cq(struct bce_device *dev, int qid, u32 el_count);
void bce_get_cq_memcfg(struct bce_queue_cq *cq, struct bce_queue_memcfg *cfg);
void bce_free_cq(struct bce_device *dev, struct bce_queue_cq *cq);
void bce_handle_cq_completions(struct bce_device *dev, struct bce_queue_cq *cq);

struct bce_queue_sq *bce_alloc_sq(struct bce_device *dev, int qid, u32 el_size, u32 el_count,
        bce_sq_completion compl, void *userdata);
void bce_get_sq_memcfg(struct bce_queue_sq *sq, struct bce_queue_cq *cq, struct bce_queue_memcfg *cfg);
void bce_free_sq(struct bce_device *dev, struct bce_queue_sq *sq);

struct bce_queue_cmdq *bce_alloc_cmdq(struct bce_device *dev, int qid, u32 el_count);
void bce_free_cmdq(struct bce_device *dev, struct bce_queue_cmdq *cmdq);

u32 bce_cmd_register_queue(struct bce_queue_cmdq *cmdq, struct bce_queue_memcfg *cfg, const char *name, bool isdirin);
u32 bce_cmd_unregister_memory_queue(struct bce_queue_cmdq *cmdq, u16 qid);
u32 bce_cmd_flush_memory_queue(struct bce_queue_cmdq *cmdq, u16 qid);

#endif //BCEDRIVER_MAILBOX_H
