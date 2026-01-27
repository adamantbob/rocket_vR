/// Assigns names to hardware resources and verifies at compile-time that no resource is assigned twice.
///
/// # Example
/// ```rust
/// assign_resources!(p => {
///     WIFI_PWR: PIN_23,
///     WIFI_CS: PIN_25,
/// });
/// ```
#[macro_export]
macro_rules! assign_resources {
    ($( $group_name:ident { $($name:ident: $field:ident),* $(,)? } )*) => {
        // 1. Structs now store Peri handles directly
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
                            // We use .into() here ONCE inside the macro
                            // so it isn't needed in the main code.
                            $( $name: p.$field.into(), )*
                        },
                    )*
                }
            }
        }
    };
}
// A set of macros to handle logging for both defmt and log.
// Log to both in order to enable in the field debugging,
// Also keeping the testing version as close to the release as possible.
#[macro_export]
macro_rules! info {
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
    };
}

#[macro_export]
macro_rules! error {
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

        defmt::info!(
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
