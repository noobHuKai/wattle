import { createFileRoute, redirect } from "@tanstack/react-router";

export const Route = createFileRoute("/_app/")({
  loader: () => {
    redirect({
      to: "/dashboard",
      throw: true,
    });
  },
});
