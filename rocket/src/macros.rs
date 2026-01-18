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
