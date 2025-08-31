import { createFileRoute, Outlet } from '@tanstack/react-router'

function ControlFlowLayout() {
  return <Outlet />
}

export const Route = createFileRoute('/_control')({
  component: ControlFlowLayout,
})
