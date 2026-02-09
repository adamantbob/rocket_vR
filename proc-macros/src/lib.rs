use proc_macro::TokenStream;
use quote::quote;
use syn::{Expr, ItemFn, parse_macro_input};

// A proc macro to wrap embassy tasks with defmt tracking
// This is expected to be used with the utilization module
// The task ID should be unique.
// Usage: #[tracked_task(TaskId::MyTask)]

#[proc_macro_attribute]
pub fn tracked_task(args: TokenStream, input: TokenStream) -> TokenStream {
    let mut item_fn = parse_macro_input!(input as ItemFn);
    let task_id_str = args.to_string();
    let task_id_expr: Expr = syn::parse_str(&task_id_str).expect("Need a TaskId argument");

    let block = &item_fn.block;

    // Rewrite the block to track the execution of the existing code
    // We don't create a new async block, we just wrap the existing logic
    let new_block = quote! {
        {
            use crate::utilization::TrackedExt;
            // We use a dummy async block just to hook into the .tracked() trait
            // then immediately await it so the function stays a single future.
            async { #block }.tracked(#task_id_expr).await
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
