import { useState } from 'react';
import { AudioRecorder } from '@/components/AudioRecorder';
import { transcribeAudio, processWithAgent } from '@/lib/api';
import './App.css';
import { Button } from '@/components/ui/button';
import { Toaster, toast } from "sonner";

function App() {
  const [inputText, setInputText] = useState<string>('');
  const [agentResult, setAgentResult] = useState<string | null>(null);
  const [isProcessing, setIsProcessing] = useState(false);

  const handleAudioRecorded = async (audioBlob: Blob) => {
    try {
      setIsProcessing(true);
      // Step 1: Transcribe the audio
      const transcriptionResult = await transcribeAudio(audioBlob);
      setInputText(transcriptionResult.text); // Populate the textarea with the transcription

      // Step 2: Process the transcription with the agent
      const agentResponse = await processWithAgent(transcriptionResult.text);
      setAgentResult(agentResponse.result);

      // Notify the user when the agent is done
      toast.success("Agent processing completed successfully!");
    } catch (error) {
      console.error('Error processing audio:', error);
      setInputText('Error transcribing audio.');
      setAgentResult('Error processing transcription.');
      toast.error("An error occurred during processing.");
    } finally {
      setIsProcessing(false);
    }
  };

  const handleTextSubmit = async () => {
    if (!inputText.trim()) return;

    try {
      setIsProcessing(true);

      // Process the input text with the agent
      const agentResponse = await processWithAgent(inputText);
      setAgentResult(agentResponse.result);

      // Notify the user when the agent is done
      toast.success("Agent processing completed successfully!");
    } catch (error) {
      console.error('Error processing text:', error);
      setAgentResult('Error processing text.');
      toast.error("An error occurred during processing.");
    } finally {
      setIsProcessing(false);
    }
  };

  return (
    <div className="flex flex-col items-center justify-center min-h-screen bg-white">
      <Toaster />
      <h1 className="text-3xl font-bold mb-6 animate-fade-in">Voice Command Center</h1>

      <AudioRecorder onAudioRecorded={handleAudioRecorded} />

      <div className="w-full max-w-md mt-6">
        <textarea
          className="w-full p-3 border border-gray-300 rounded-lg shadow-sm focus:outline-none focus:ring-2 focus:ring-blue-500 focus:border-blue-500 transition"
          rows={3}
          placeholder="Type your command here or record audio..."
          value={inputText}
          onChange={(e) => setInputText(e.target.value)}
        />
        <Button
          className="mt-3 w-full hover:cursor-pointer transition-all duration-300 active:scale-95"
          onClick={handleTextSubmit}
          disabled={isProcessing}
        >
          {isProcessing ? 'Processing...' : 'Submit'}
        </Button>
      </div>

      {agentResult && (
        <div className="w-full text-center mt-6 animate-slide-up">
          <h3 className="text-lg font-semibold">Agent Result</h3>
          <p className="text-gray-700 mt-2">{agentResult}</p>
        </div>
      )}
    </div>
  );
}

export default App;
