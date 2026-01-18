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
