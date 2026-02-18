//! # Actor Framework for Embedded Async Systems
//!
//! Zero-allocation actor infrastructure built on Embassy.
//! Provides typed mailboxes, system event bus, and priority-aware scheduling
//! so that control is always responsive — even during long-running motor moves.

#![no_std]

pub mod mailbox;
pub mod event_bus;
pub mod select;

pub use mailbox::{Mailbox, MailboxSender, MailboxReceiver};
pub use event_bus::{EventBus, EventPublisher, EventSubscriber};

use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;

/// Default mutex type: safe from both thread and interrupt context on single-core Cortex-M.
pub type Mutex = CriticalSectionRawMutex;
