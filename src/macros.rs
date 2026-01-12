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
    ($p:expr => { $($name:ident: $field:ident),* $(,)? }) => {
        // Compile-time duplicate detection
        const _: () = {
            let fields = [$(stringify!($field)),*];
            let mut i = 0;
            while i < fields.len() {
                let mut j = i + 1;
                while j < fields.len() {
                    // Compare strings byte by byte at compile time.
                    // This ensures no two names are mapped to the same hardware peripheral field.
                    let a = fields[i].as_bytes();
                    let b = fields[j].as_bytes();
                    if a.len() == b.len() {
                        let mut k = 0;
                        let mut match_found = true;
                        while k < a.len() {
                            if a[k] != b[k] {
                                match_found = false;
                                break;
                            }
                            k += 1;
                        }
                        if match_found {
                            panic!("Resource conflict detected in assign_resources!");
                        }
                    }
                    j += 1;
                }
                i += 1;
            }
        };

        // Generate the bindings: let Name = Peripherals.Field;
        $(
            #[allow(non_snake_case)]
            let $name = $p.$field;
        )*
    };
}
