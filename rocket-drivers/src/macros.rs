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
