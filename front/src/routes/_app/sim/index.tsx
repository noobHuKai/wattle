import { createFileRoute } from "@tanstack/react-router";

export const Route = createFileRoute("/_app/sim/")({
  component: App,
});

import data from "./data.json";
import { DataTable } from "./-components/data-table";

function App() {
  return (
    <div>
      <DataTable data={data} />
    </div>
  );
}
