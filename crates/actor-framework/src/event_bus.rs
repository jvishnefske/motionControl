//! System-wide event bus — broadcast channel for cross-cutting concerns.
//!
//! Events like emergency stop, thermal runaway, and homing complete
//! are published once and delivered to every subscriber.

use super::Mutex;
use embassy_sync::pubsub::PubSubChannel;

/// A broadcast event bus with `CAP` message slots, `SUBS` subscribers, and `PUBS` publishers.
pub type EventBus<E, const CAP: usize, const SUBS: usize, const PUBS: usize> =
    PubSubChannel<Mutex, E, CAP, SUBS, PUBS>;

/// Publisher handle for sending events.
pub type EventPublisher<'a, E, const CAP: usize, const SUBS: usize, const PUBS: usize> =
    embassy_sync::pubsub::Publisher<'a, Mutex, E, CAP, SUBS, PUBS>;

/// Subscriber handle for receiving events.
pub type EventSubscriber<'a, E, const CAP: usize, const SUBS: usize, const PUBS: usize> =
    embassy_sync::pubsub::Subscriber<'a, Mutex, E, CAP, SUBS, PUBS>;
