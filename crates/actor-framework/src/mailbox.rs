//! Typed actor mailbox — a bounded async channel that serves as an actor's inbox.
//!
//! Each actor owns a `MailboxReceiver` and processes messages sequentially.
//! Any number of senders can push messages into the mailbox with backpressure.

use embassy_sync::channel::Channel;
use super::Mutex;

/// A statically-allocated actor mailbox with capacity `N`.
pub type Mailbox<M, const N: usize> = Channel<Mutex, M, N>;

/// Sender half — cloneable, can be shared across tasks.
pub type MailboxSender<'a, M, const N: usize> =
    embassy_sync::channel::Sender<'a, Mutex, M, N>;

/// Receiver half — owned by exactly one actor task.
pub type MailboxReceiver<'a, M, const N: usize> =
    embassy_sync::channel::Receiver<'a, Mutex, M, N>;
