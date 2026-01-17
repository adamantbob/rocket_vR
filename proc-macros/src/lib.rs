use proc_macro::TokenStream;
use quote::quote;
use syn::{Expr, ItemFn, parse_macro_input};

// A proc macro to wrap embassy tasks with defmt tracking
// This is expected to be used with the utilization module
// The task ID should be unique.
// Usage: #[tracked_task(TaskId::MyTask)]

#[proc_macro_attribute]
pub fn tracked_task(args: TokenStream, input: TokenStream) -> TokenStream {
    let item_fn = parse_macro_input!(input as ItemFn);

    // Parse the ID from the macro arguments: #[tracked_task(TaskId::MyTask)]
    let task_id = args.to_string();
    let task_id_expr: Expr = syn::parse_str(&task_id).expect("Need a TaskId argument");

    let block = &item_fn.block;
    let attrs = &item_fn.attrs;
    let vis = &item_fn.vis;
    let sig = &item_fn.sig;

    // Wrap the original body
    let new_body = quote! {
        {
            async move #block
                .tracked(#task_id_expr)
                .await
        }
    };

    // Reconstruct the function
    let output = quote! {
        #[embassy_executor::task]
        #(#attrs)*
        #vis #sig #new_body
    };

    output.into()
}
