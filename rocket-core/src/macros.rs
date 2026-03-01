#[macro_export]
macro_rules! info {
    ($($arg:tt)*) => {
        ::defmt::info!($($arg)*);
        ::log::info!($($arg)*);
        $crate::log_to_sd!($crate::log::LogLevel::Info, $($arg)*);
    };
}

#[macro_export]
macro_rules! local_info {
    ($($arg:tt)*) => {
        ::defmt::info!($($arg)*);
        ::log::info!($($arg)*);
    };
}

#[macro_export]
macro_rules! warn {
    ($($arg:tt)*) => {
        ::defmt::warn!($($arg)*);
        ::log::warn!($($arg)*);
        $crate::log_to_sd!($crate::log::LogLevel::Warn, $($arg)*);
    };
}

#[macro_export]
macro_rules! local_warn {
    ($($arg:tt)*) => {
        ::defmt::warn!($($arg)*);
        ::log::warn!($($arg)*);
    };
}

#[macro_export]
macro_rules! error {
    ($($arg:tt)*) => {
        ::defmt::error!($($arg)*);
        ::log::error!($($arg)*);
        $crate::log_to_sd!($crate::log::LogLevel::Error, $($arg)*);
    };
}

#[macro_export]
macro_rules! local_error {
    ($($arg:tt)*) => {
        ::defmt::error!($($arg)*);
        ::log::error!($($arg)*);
    };
}

#[macro_export]
macro_rules! debug {
    ($($arg:tt)*) => {
        ::defmt::debug!($($arg)*);
        ::log::debug!($($arg)*);
    };
}

#[macro_export]
macro_rules! trace {
    ($($arg:tt)*) => {
        ::defmt::trace!($($arg)*);
        ::log::trace!($($arg)*);
    };
}

#[macro_export]
macro_rules! log_to_sd {
    ($level:expr, $($arg:tt)*) => {
        {
            use core::fmt::Write;
            use core::sync::atomic::Ordering;
            let mut s = ::heapless::String::<32>::new();
            if let Ok(_) = write!(s, $($arg)*) {
                if let Err(_) = $crate::external_log::LOG_CHANNEL.try_send(
                    $crate::external_log::LogEntry::Log($level, s)
                ) {
                    $crate::external_log::DROPPED_LOGS.fetch_add(1, Ordering::Relaxed);
                }
            }
        }
    };
}

#[macro_export]
macro_rules! log_triad_fixed_width {
    ($label:expr, $units:expr) => {{
        let mut parts = [(0i32, 0i32, 0i32, 0i32, 0i32); 3];

        for i in 0..3 {
            let val = $units[i];
            let sign = if val < 0 { 1 } else { 0 };
            let abs_val = val.abs();
            let whole = abs_val / 1000;
            let rem = abs_val % 1000;

            let d1 = rem / 100;
            let d2 = (rem / 10) % 10;
            let d3 = rem % 10;

            parts[i] = (sign, whole, d1, d2, d3);
        }

        defmt::debug!(
            "{} | X:{}{}.{}{}{} | Y:{}{}.{}{}{} | Z:{}{}.{}{}{}",
            $label,
            if parts[0].0 == 1 { "-" } else { " " },
            parts[0].1,
            parts[0].2,
            parts[0].3,
            parts[0].4,
            if parts[1].0 == 1 { "-" } else { " " },
            parts[1].1,
            parts[1].2,
            parts[1].3,
            parts[1].4,
            if parts[2].0 == 1 { "-" } else { " " },
            parts[2].1,
            parts[2].2,
            parts[2].3,
            parts[2].4
        );
    }};
}

#[macro_export]
macro_rules! define_utilization_tasks {
    ($($name:ident [$core:expr]),*) => {
        #[allow(non_camel_case_types)]
        pub enum TaskIdEnum {
            $($name),*
        }

        /// Human-readable names of the registered tasks.
        pub const TASK_NAMES: &[&'static str] = &[ $( stringify!($name) ),* ];

        /// Core assignments for the registered tasks.
        pub const TASK_CORES: &[u8] = &[ $( $core ),* ];

        // Compile-time assertion that we haven't exceeded MAX_TASKS.
        const _: () = assert!(TASK_NAMES.len() <= $crate::utilization::MAX_TASKS, "Too many tasks registered for utilization tracking!");

        $(
            #[allow(non_upper_case_globals)]
            /// Unique identifier for the task's utilization tracking.
            pub const $name: $crate::utilization::TaskId = $crate::utilization::TaskId(TaskIdEnum::$name as usize);
        )*
    }
}

#[macro_export]
macro_rules! spawn_tracked {
    ($spawner:expr, $id:expr, $task_fn:ident($($arg:expr),*)) => {
        $spawner.spawn($task_fn($($arg,)* $id).unwrap())
    }
}

/// Assigns names to hardware resources and verifies at compile-time that no resource is assigned twice.
#[macro_export]
macro_rules! assign_resources {
    ($( $group_name:ident { $($name:ident: $field:ident),* $(,)? } )*) => {
        $(
            pub struct $group_name {
                $( pub $name: embassy_rp::Peri<'static, embassy_rp::peripherals::$field>, )*
            }
        )*

        #[allow(non_snake_case)]
        pub struct AssignedResources {
            $( pub $group_name: $group_name, )*
        }

        impl AssignedResources {
            pub fn take(p: embassy_rp::Peripherals) -> Self {
                Self {
                    $(
                        $group_name: $group_name {
                            $( $name: p.$field.into(), )*
                        },
                    )*
                }
            }
        }
    };
}
