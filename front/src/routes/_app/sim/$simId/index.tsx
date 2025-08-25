import { createFileRoute } from "@tanstack/react-router";
import { WebViewer } from "@rerun-io/web-viewer";
import { useEffect, useRef, useState } from "react";

export const Route = createFileRoute("/_app/sim/$simId/")({
  component: App,
});

function App() {
  const { simId } = Route.useParams();
  // 示例 RRD 数据 URL，您可以根据 simId 动态构建
  const [rrdUrl, setRrdUrl] = useState<string | null>(
    "https://app.rerun.io/version/0.24.1/examples/arkit_scenes.rrd"
  );
  const viewerRef = useRef<HTMLDivElement>(null);
  const webViewerRef = useRef<WebViewer | null>(null);

  useEffect(() => {
    const initViewer = async () => {
      if (viewerRef.current && !webViewerRef.current) {
        try {
          const viewer = new WebViewer();
          webViewerRef.current = viewer;
          
          // 启动 viewer
          await viewer.start(rrdUrl, viewerRef.current, {});
          
          // 注册选择变化回调
          viewer.on("selection_change", (event) => {
            for (const item of event.items) {
              if (item.type === "entity") {
                console.log("Selected entity:", item.entity_path);
              }
            }
          });
        } catch (error) {
          console.error("Failed to initialize Rerun viewer:", error);
        }
      }
    };

    initViewer();

    // 清理函数
    return () => {
      if (webViewerRef.current) {
        webViewerRef.current.stop();
        webViewerRef.current = null;
      }
    };
  }, [rrdUrl]);

  const handleOpenRecording = () => {
    if (webViewerRef.current && rrdUrl) {
      webViewerRef.current.open(rrdUrl);
    }
  };

  const handleCloseRecording = () => {
    if (webViewerRef.current && rrdUrl) {
      webViewerRef.current.close(rrdUrl);
    }
  };

  return (
    <div className="w-full h-full flex flex-col">
      <div className="p-4 border-b">
        <h1 className="text-2xl font-bold">Simulation Viewer - {simId}</h1>
        <div className="mt-4 flex gap-2">
          <button
            onClick={handleOpenRecording}
            className="px-4 py-2 bg-blue-500 text-white rounded hover:bg-blue-600"
          >
            打开录制
          </button>
          <button
            onClick={handleCloseRecording}
            className="px-4 py-2 bg-red-500 text-white rounded hover:bg-red-600"
          >
            关闭录制
          </button>
          <input
            type="text"
            value={rrdUrl || ""}
            onChange={(e) => setRrdUrl(e.target.value || null)}
            placeholder="输入 RRD 文件 URL"
            className="px-3 py-2 border rounded flex-1 max-w-md"
          />
        </div>
      </div>
      <div className="flex-1 relative">
        <div
          ref={viewerRef}
          id="viewer-container"
          className="w-full h-full"
          style={{ minHeight: "600px" }}
        />
      </div>
    </div>
  );
}
