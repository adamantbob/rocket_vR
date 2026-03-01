use proc_macro::TokenStream;
use quote::quote;
use syn::{ItemFn, parse_macro_input};

// A proc macro to wrap embassy tasks with defmt tracking
// This is expected to be used with the utilization module
// The task ID should be unique.
// Usage: #[tracked_task(TaskId::MyTask)]

#[proc_macro_attribute]
pub fn tracked_task(_args: TokenStream, input: TokenStream) -> TokenStream {
    let mut item_fn = parse_macro_input!(input as ItemFn);

    // 1. Inject 'utilization_id: TaskId' into the function arguments as the last parameter
    let id_param: syn::FnArg = syn::parse_quote! {
        utilization_id: ::rocket_core::utilization::TaskId
    };
    item_fn.sig.inputs.push(id_param);

    let block = &item_fn.block;

    // 2. Rewrite the block to track the execution using the injected 'utilization_id'
    let new_block = quote! {
        {
            use ::rocket_core::utilization::TrackedExt;
            TrackedExt::tracked(async { #block }, utilization_id).await
        }
    };

    item_fn.block = syn::parse2(new_block).expect("Failed to parse wrapper block");

    quote! {
        #item_fn
    }
    .into()
}

#[proc_macro_derive(TelemetryPayload)]
pub fn derive_telemetry_payload(input: TokenStream) -> TokenStream {
    let input = parse_macro_input!(input as syn::DeriveInput);
    let name = input.ident;

    let fields = if let syn::Data::Struct(syn::DataStruct {
        fields: syn::Fields::Named(syn::FieldsNamed { named, .. }),
        ..
    }) = input.data
    {
        named
    } else {
        panic!("TelemetryPayload can only be derived for structs with named fields");
    };

    // Build the Header string, skipping 'tickstamp' if it exists in the struct
    let header_string = fields
        .iter()
        .map(|f| f.ident.as_ref().unwrap().to_string())
        .filter(|n| n != "tickstamp")
        .collect::<Vec<_>>()
        .join(",");

    let expanded = quote! {
        impl #name {
            pub const CSV_HEADER: &'static str = #header_string;
        }
    };
    TokenStream::from(expanded)
}
