#include "pthread_impl.h"

#ifdef WASIX_LIBC_DEBUG_PTHREAD_MUTEX
#include <stdio.h>

#ifdef __wasilibc_unmodified_upstream
#define WASILIBC_UNMODIFIED_UPSTREAM_DEFINED 1
#else
#define WASILIBC_UNMODIFIED_UPSTREAM_DEFINED 0
#endif

static void debug_pthread_mutex_trylock_error(pthread_mutex_t *m, pthread_t self,
	const char *branch, int err, int old, int own, int tid)
{
	fprintf(stderr,
		"wasix-libc pthread_mutex_trylock error: "
		"branch=%s err=%d m=%p m_type=%d m_lock=%d m_waiters=%d "
		"old=%d own=%d self_tid=%d tid=%d "
		"__wasilibc_unmodified_upstream=%d\n",
		branch, err, (void *)m, m->_m_type, m->_m_lock, m->_m_waiters,
		old, own, self ? self->tid : -1, tid,
		WASILIBC_UNMODIFIED_UPSTREAM_DEFINED);
}

#define DEBUG_PTHREAD_MUTEX_TRYLOCK_ERROR(m, self, branch, err, old, own, tid) \
	debug_pthread_mutex_trylock_error((m), (self), (branch), (err), (old), (own), (tid))
#else
#define DEBUG_PTHREAD_MUTEX_TRYLOCK_ERROR(m, self, branch, err, old, own, tid) ((void)0)
#endif

int __pthread_mutex_trylock_owner(pthread_mutex_t *m)
{
	int old, own;
	int type = m->_m_type;
	pthread_t self = __pthread_self();
	int tid = self->tid;

	old = m->_m_lock;
	own = old & 0x3fffffff;
	if (own == tid) {
		if ((type&8) && m->_m_count<0) {
			old &= 0x40000000;
			m->_m_count = 0;
			goto success;
		}
		if ((type&3) == PTHREAD_MUTEX_RECURSIVE) {
			if ((unsigned)m->_m_count >= INT_MAX) {
				DEBUG_PTHREAD_MUTEX_TRYLOCK_ERROR(m, self, "recursive-count-overflow", EAGAIN, old, own, tid);
				return EAGAIN;
			}
			m->_m_count++;
			return 0;
		}
	}
	if (own == 0x3fffffff) {
		DEBUG_PTHREAD_MUTEX_TRYLOCK_ERROR(m, self, "owner-sentinel", ENOTRECOVERABLE, old, own, tid);
		return ENOTRECOVERABLE;
	}
	if (own || (old && !(type & 4))) {
		DEBUG_PTHREAD_MUTEX_TRYLOCK_ERROR(m, self, "busy", EBUSY, old, own, tid);
		return EBUSY;
	}

	if (type & 128) {
		if (!self->robust_list.off) {
			self->robust_list.off = (char*)&m->_m_lock-(char *)&m->_m_next;
#ifdef __wasilibc_unmodified_upstream
			__syscall(SYS_set_robust_list, &self->robust_list, 3*sizeof(long));
#endif
		}
		if (m->_m_waiters) tid |= 0x80000000;
		self->robust_list.pending = &m->_m_next;
	}
	tid |= old & 0x40000000;

	if (a_cas(&m->_m_lock, old, tid) != old) {
		self->robust_list.pending = 0;
		if ((type&12)==12 && m->_m_waiters) {
			DEBUG_PTHREAD_MUTEX_TRYLOCK_ERROR(m, self, "cas-failed-robust-pi-waiters", ENOTRECOVERABLE, old, own, tid);
			return ENOTRECOVERABLE;
		}
		DEBUG_PTHREAD_MUTEX_TRYLOCK_ERROR(m, self, "cas-failed", EBUSY, old, own, tid);
		return EBUSY;
	}

success:
	if ((type&8) && m->_m_waiters) {
		int priv = (type & 128) ^ 128;
#ifdef __wasilibc_unmodified_upstream
		__syscall(SYS_futex, &m->_m_lock, FUTEX_UNLOCK_PI|priv);
#endif
		self->robust_list.pending = 0;
		if (type&4) {
			DEBUG_PTHREAD_MUTEX_TRYLOCK_ERROR(m, self, "pi-waiters-robust", ENOTRECOVERABLE, old, old & 0x3fffffff, tid);
			return ENOTRECOVERABLE;
		}
		DEBUG_PTHREAD_MUTEX_TRYLOCK_ERROR(m, self, "pi-waiters", EBUSY, old, old & 0x3fffffff, tid);
		return EBUSY;
	}

	volatile void *next = self->robust_list.head;
	m->_m_next = next;
	m->_m_prev = &self->robust_list.head;
	if (next != &self->robust_list.head) *(volatile void *volatile *)
		((char *)next - sizeof(void *)) = &m->_m_next;
	self->robust_list.head = &m->_m_next;
	self->robust_list.pending = 0;

	if (old) {
		m->_m_count = 0;
		DEBUG_PTHREAD_MUTEX_TRYLOCK_ERROR(m, self, "owner-dead", EOWNERDEAD, old, old & 0x3fffffff, tid);
		return EOWNERDEAD;
	}

	return 0;
}

int __pthread_mutex_trylock(pthread_mutex_t *m)
{
	if ((m->_m_type&15) == PTHREAD_MUTEX_NORMAL) {
		int err = a_cas(&m->_m_lock, 0, EBUSY) & EBUSY;
#ifdef WASIX_LIBC_DEBUG_PTHREAD_MUTEX
		if (err) {
			pthread_t self = __pthread_self();
			int old = m->_m_lock;
			DEBUG_PTHREAD_MUTEX_TRYLOCK_ERROR(m, self, "normal-busy", err, old, old & 0x3fffffff, self->tid);
		}
#endif
		return err;
	}
	return __pthread_mutex_trylock_owner(m);
}

weak_alias(__pthread_mutex_trylock, pthread_mutex_trylock);
